from pathlib import Path

import numpy as np
import openmdao.api as om


class AerodynamicsComp(om.ExplicitComponent):
    """
    Simple fixed-airfoil aerodynamic model.

    Structural twist reduces the effective angle of attack: alpha_eff = alpha_geo - theta

    Lift follows thin-airfoil theory: CL = 2*pi*alpha_eff

    Drag uses a simple parabolic polar: CD = CD0 + k*CL^2

    The lift acting at an offset e from the elastic axis
    creates the aerodynamic twisting moment.
    """

    def setup(self):
        self.add_input("alpha_geo", val=np.deg2rad(5.0), units="rad")
        self.add_input("theta", val=0.0, units="rad")

        self.add_input("rho", val=1.225, units="kg/m**3")
        self.add_input("U", val=30.0, units="m/s")
        self.add_input("S", val=0.1, units="m**2")
        self.add_input("e", val=0.05, units="m")

        self.add_input("CD0", val=0.012)
        self.add_input("k_drag", val=0.02)

        self.add_output("alpha_eff", val=np.deg2rad(5.0), units="rad")
        self.add_output("CL", val=0.0)
        self.add_output("CD", val=0.0)

        self.add_output("lift", val=0.0, units="N")
        self.add_output("M_aero", val=0.0, units="N*m")

    def setup_partials(self):
        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        alpha_eff = inputs["alpha_geo"] - inputs["theta"]

        CL = 2.0 * np.pi * alpha_eff

        CD = (inputs["CD0"]+ inputs["k_drag"] * CL**2)

        q = 0.5 * inputs["rho"] * inputs["U"]**2

        lift = q * inputs["S"] * CL

        M_aero = lift * inputs["e"]

        outputs["alpha_eff"] = alpha_eff
        outputs["CL"] = CL
        outputs["CD"] = CD
        outputs["lift"] = lift
        outputs["M_aero"] = M_aero


class StructuralTorsionComp(om.ExplicitComponent):
    """
    Lumped torsional structural model:

        theta = M_aero / K_theta
    """

    def setup(self):
        self.add_input("M_aero", val=0.0, units="N*m")

        self.add_input("K_theta",val=30.0,units="N*m/rad")

        self.add_output("theta", val=0.0, units="rad")

    def setup_partials(self):
        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        outputs["theta"] = (inputs["M_aero"] / inputs["K_theta"])


class PerformanceComp(om.ExplicitComponent):
    """Compute aerodynamic efficiency."""

    def setup(self):
        self.add_input("CL", val=0.0)
        self.add_input("CD", val=0.02)

        self.add_output("L_D", val=0.0)

    def compute(self, inputs, outputs):
        outputs["L_D"] = inputs["CL"] / inputs["CD"]


class AeroelasticCycle(om.Group):
    """
    Coupled aerodynamic-structural group.

        theta -> aerodynamics -> M_aero -> structure -> theta
    """

    def initialize(self):
        self.options.declare("solver_kind",default="nlbgs",values=("nlbgs", "nlbgs_aitken","newton","broyden"),)

        self.options.declare("iprint",default=0,types=int)

    def setup(self):
        self.add_subsystem("aero",AerodynamicsComp())

        self.add_subsystem("structure",StructuralTorsionComp())

        # Forward connection:
        # aerodynamics -> structure
        self.connect("aero.M_aero","structure.M_aero")

        # FEEDBACK CONNECTION:
        # structure -> aerodynamics
        self.connect("structure.theta","aero.theta")

        solver = om.NonlinearBlockGS()

        if self.options["solver_kind"] == "nlbgs":
            solver = om.NonlinearBlockGS()
        elif self.options["solver_kind"] == "nlbgs_aitken":
            solver = om.NonlinearBlockGS()
            solver.options["use_aitken"] = True

        elif self.options["solver_kind"] == "newton":
            solver = om.NewtonSolver(solve_subsystems=False)
            self.linear_solver = om.DirectSolver()
        
        elif self.options["solver_kind"] == "broyden":
            solver = om.BroydenSolver()
            self.linear_solver = om.DirectSolver()

        else:
            raise ValueError(f"Unknown solver_kind: {self.options['solver_kind']}")
      
        
        solver.options["maxiter"] = 50
        solver.options["atol"] = 1.0e-10
        solver.options["rtol"] = 1.0e-10
        solver.options["iprint"] = self.options["iprint"]

        self.nonlinear_solver = solver


class AeroelasticModel(om.Group):

    def initialize(self):
        self.options.declare("solver_kind",default="nlbgs",values=("nlbgs", "nlbgs_aitken","newton", "broyden"),)

        self.options.declare("iprint",default=0,types=int)

    def setup(self):

        # ---------------------------------------------
        # Independent inputs
        # ---------------------------------------------
        inputs = om.IndepVarComp()

        inputs.add_output("alpha_geo",val=np.deg2rad(5.0),units="rad")

        inputs.add_output("rho",val=1.225,units="kg/m**3")

        inputs.add_output("U",val=30.0,units="m/s")

        inputs.add_output("S",val=0.1,units="m**2")

        inputs.add_output("e",val=0.05, units="m")

        inputs.add_output("K_theta",val=30.0,units="N*m/rad")

        inputs.add_output("CD0",val=0.012)

        inputs.add_output("k_drag",val=0.02)

        self.add_subsystem("inputs",inputs,promotes_outputs=["*"])

        # ---------------------------------------------
        # Coupled aeroelastic system
        # ---------------------------------------------
        cycle = AeroelasticCycle(solver_kind=self.options["solver_kind"],iprint=self.options["iprint"],)

        self.add_subsystem("cycle",cycle)

        self.connect("alpha_geo", "cycle.aero.alpha_geo")
        self.connect("rho", "cycle.aero.rho")
        self.connect("U", "cycle.aero.U")
        self.connect("S", "cycle.aero.S")
        self.connect("e", "cycle.aero.e")
        self.connect("CD0", "cycle.aero.CD0")
        self.connect("k_drag", "cycle.aero.k_drag")

        self.connect("K_theta","cycle.structure.K_theta",)

        # ---------------------------------------------
        # Performance calculation outside cycle
        # ---------------------------------------------
        self.add_subsystem("performance",PerformanceComp())

        self.connect("cycle.aero.CL","performance.CL")

        self.connect("cycle.aero.CD","performance.CD")


def build_problem(solver_kind="nlbgs",iprint=0):
    prob = om.Problem(
        model=AeroelasticModel(solver_kind=solver_kind,iprint=iprint),
        reports=False)

    # check=True asks OpenMDAO to check the model configuration.
    prob.setup(check=True)

    return prob


def main():
    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)

    prob = build_problem(solver_kind="nlbgs",iprint=2)

    prob.run_model()

    theta_deg = np.rad2deg(prob.get_val("cycle.structure.theta",units="rad")).item()

    alpha_eff_deg = np.rad2deg(prob.get_val("cycle.aero.alpha_eff",units="rad",)).item()

    CL = prob.get_val("cycle.aero.CL").item()
    CD = prob.get_val("cycle.aero.CD").item()
    L_D = prob.get_val("performance.L_D").item()

    print("\nCONVERGED AEROELASTIC STATE")
    print("--------------------------------")
    print(f"Twist theta       = {theta_deg:.6f} deg")
    print(f"Effective alpha   = {alpha_eff_deg:.6f} deg")
    print(f"CL                = {CL:.6f}")
    print(f"CD                = {CD:.6f}")
    print(f"L/D               = {L_D:.6f}")

    # Generate assignment N2 diagram.
    n2_path = output_dir / "aeroelastic_n2.html"

    om.n2(prob,outfile=str(n2_path),show_browser=False,title="Coupled Airfoil Aeroelastic Analysis",)

    print(f"\nN2 diagram written to: {n2_path}")

    prob.cleanup()


if __name__ == "__main__":
    main()