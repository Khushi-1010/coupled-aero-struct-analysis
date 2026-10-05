# coupled-aero-struct-analysis

An OpenMDAO aeroelastic analysis of a fixed airfoil section: a two-component
aerodynamics/structural-torsion feedback loop, solved as a coupled fixed-point
problem and compared across four nonlinear solver strategies.

## Project objective

Model the steady aeroelastic equilibrium of a rigid-planform airfoil whose
aerodynamic loading twists a torsionally-flexible section, which in turn changes
the loading. The two physical effects (`AerodynamicsComp`, `StructuralTorsionComp`)
are wired into an explicit two-way coupling (`AeroelasticCycle`) that must be
solved iteratively, while a downstream `PerformanceComp` reports L/D from the
converged state without itself being part of that iteration. The project then
compares how differently nonlinear solvers handle this coupling:
**NonlinearBlockGS (NLBGS)**, **NLBGS with Aitken relaxation**, **Newton**, and
**Broyden** -- recording every solver iteration and plotting residual histories
head to head.

## Governing aeroelastic equations

```
alpha_eff = alpha_geo - theta                  effective angle of attack (rad)
CL        = 2*pi * alpha_eff                   lift coefficient (thin-airfoil slope)
CD        = CD0 + k_drag * CL**2               parabolic drag polar
q         = 0.5 * rho * U**2                   dynamic pressure
lift      = q * S * CL                         lift force (N)
M_aero    = lift * e                           aerodynamic moment about the elastic axis (N*m)
theta     = M_aero / K_theta                   elastic twist from torsional stiffness (rad)
L_D       = CL / CD                            lift-to-drag ratio
```

`e` is the lift's moment arm from the elastic axis, and `K_theta` is the
section's torsional stiffness. Twist here is a **washout** effect: increasing
`theta` *reduces* `alpha_eff` (`alpha_eff = alpha_geo - theta`), which reduces
`CL` and hence `M_aero` -- the loop is a damped, self-equilibrating (negative)
feedback, not a torsional-divergence (positive-feedback) configuration. It
converges for any positive `K_theta`; it does not model a divergence boundary.

## Components

### `AerodynamicsComp`

Computes the aerodynamic state from geometric angle of attack and the current
twist:

- **Inputs:** `alpha_geo`, `theta` (rad); `rho`, `U`, `S`, `e`, `CD0`, `k_drag`
  (freestream and geometric/polar parameters)
- **Outputs:** `alpha_eff`, `CL`, `CD`, `lift`, `M_aero`

It is the only component that reads `theta` and the only one that produces
`M_aero` -- both ends of the feedback connection described below. Partials are
finite-differenced (`setup_partials` uses `method="fd"`).

### `StructuralTorsionComp`

A single-degree-of-freedom torsional spring:

- **Inputs:** `M_aero` (N*m), `K_theta` (N*m/rad)
- **Output:** `theta` (rad), via `theta = M_aero / K_theta`

Also finite-differenced. It is the only component that produces `theta`.

### `PerformanceComp`

Takes the converged `CL`/`CD` and reports `L_D = CL / CD`. It has no connection
back into the cycle -- it is evaluated once, after the coupled loop has already
converged.

## The feedback loop

`AeroelasticCycle` (an `om.Group`) contains just `aero` (`AerodynamicsComp`) and
`structure` (`StructuralTorsionComp`), connected both ways:

```
aero.M_aero      -> structure.M_aero     (forward: aerodynamic load into the structure)
structure.theta  -> aero.theta           (feedback: elastic twist back into the aerodynamics)
```

Because each component's output feeds the other's input, the pair forms an
algebraic cycle with no explicit solution -- it must be iterated to a
fixed point. `AeroelasticCycle` owns its own `nonlinear_solver` (and, for the
derivative-based solver choices, its own `linear_solver`) so that this
iteration is scoped to exactly these two components, not the whole model.

## OpenMDAO model hierarchy

```
AeroelasticModel                         (top-level Group)
├── inputs            (IndepVarComp, promoted: alpha_geo, rho, U, S, e,
│                       K_theta, CD0, k_drag)
├── cycle              (AeroelasticCycle Group -- the solved feedback loop)
│   ├── aero           (AerodynamicsComp)
│   └── structure      (StructuralTorsionComp)
└── performance        (PerformanceComp -- outside the cycle)
```

`inputs` feeds `cycle.aero.*` and `cycle.structure.K_theta` by explicit
`connect()` calls (not promotion). `performance` is connected from
`cycle.aero.CL` / `cycle.aero.CD`, downstream of the cycle, with no path back
into it.

## Solver methods

`AeroelasticCycle` accepts a `solver_kind` option and configures
`self.nonlinear_solver` (and `self.linear_solver` where needed) accordingly;
all four share `maxiter=50`, `atol=1e-10`, `rtol=1e-10`:

| `solver_kind` | Nonlinear solver | Linear solver | Notes |
|---|---|---|---|
| `nlbgs` | `NonlinearBlockGS()` | *(none)* | plain fixed-point Gauss-Seidel iteration |
| `nlbgs_aitken` | `NonlinearBlockGS(use_aitken=True)` | *(none)* | Gauss-Seidel with Aitken-relaxed updates |
| `newton` | `NewtonSolver(solve_subsystems=False)` | `DirectSolver()` | Newton's method; needs derivatives, hence the linear solver |
| `broyden` | `BroydenSolver()` | `DirectSolver()` | quasi-Newton; also needs a linear solver |

`model.py`'s `main()` always builds and runs the `nlbgs` case (for the smoke
test / N2 diagram). `solver_compare.py` runs all four and reports, for this
model's default parameters, convergence in **43** (NLBGS), **6** (NLBGS +
Aitken), **2** (Newton), and **4** (Broyden) iterations to the same converged
state -- confirming all four solvers agree on the answer while differing
sharply in how fast they get there.

## Generated outputs

Running `model.py` writes:

- **`outputs/aeroelastic_n2.html`** -- the N2 diagram for the `nlbgs` configuration

Running `solver_compare.py` writes:

- **`outputs/iterations_<solver>.csv`** (`nlbgs`, `nlbgs_aitken`, `newton`,
  `broyden`) -- one row per solver iteration, with absolute and relative
  residual norms, read back from each run's `SqliteRecorder` via `om.CaseReader`
- **`outputs/convergence.png`** -- residual norm vs. iteration for all four
  solvers on one semi-log plot
- **`outputs/convergence_nlbgs_vs_newton.png`**,
  **`convergence_newton_vs_broyden.png`**,
  **`convergence_nlbgs_vs_aitken.png`** -- the same comparison, pairwise
- **`outputs/solver_summary.txt`** -- iteration counts and converged
  `theta`/`alpha_eff`/`CL`/`CD`/`L_D` for each solver

## What to look for in the N2 diagram

Open `outputs/aeroelastic_n2.html` in a browser:

- The **diagonal cells**, top to bottom, are `inputs`, `cycle.aero`,
  `cycle.structure`, and `performance` -- the model hierarchy shown above,
  flattened to its leaf components.
- **Off-diagonal cells** mark data connections. Look for the pair of cells
  connecting `aero` and `structure`: one below the diagonal (`aero.M_aero ->
  structure.M_aero`, the forward path) and one *above* the diagonal
  (`structure.theta -> aero.theta`, the feedback path). A connection above the
  diagonal is exactly what marks a cycle in an N2 diagram -- data flowing
  "backward" relative to execution order.
- OpenMDAO highlights `aero` and `structure` as belonging to a solver loop
  (shown nested inside the `cycle` group, with the group's nonlinear solver
  named alongside it) -- this is the visual confirmation that the solver is
  scoped to just those two components.
- `performance` has an incoming connection from `cycle.aero` but nothing
  downstream of it and no connection back into `cycle` -- visual confirmation
  that it sits outside the solved loop, evaluated once after convergence.

## Running it

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe model.py            # solves the nlbgs case, prints the converged state, writes the N2 diagram
.\.venv\Scripts\python.exe solver_compare.py    # runs all four solvers, writes CSVs/plots/summary under outputs/
```

## Repository layout

```
model.py            AerodynamicsComp, StructuralTorsionComp, PerformanceComp,
                     AeroelasticCycle, AeroelasticModel, build_problem()
solver_compare.py    runs all four solver_kind options, records residual
                     histories, writes the comparison plots/CSVs/summary
outputs/             generated artifacts (recreated by the scripts above)
requirements.txt     openmdao[all], numpy, matplotlib
```
