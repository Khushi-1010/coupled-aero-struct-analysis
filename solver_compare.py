import csv
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import openmdao.api as om

from model import build_problem


OUTPUT_DIR = Path("outputs")


SOLVER_LABELS = {"nlbgs": "NLBGS","nlbgs_aitken": "NLBGS + Aitken","newton":"Newton","broyden": "Broyden"}


def run_solver(kind, work_dir):
    prob = build_problem(solver_kind=kind,iprint=0,)
    solver = prob.model.cycle.nonlinear_solver

    recorder_path = (work_dir / f"{kind}_cases.sql")

    recorder = om.SqliteRecorder(str(recorder_path))

    solver.add_recorder(recorder)

    solver.recording_options["record_abs_error"] = True

    solver.recording_options["record_rel_error"] = True

    prob.run_model()

    iteration_count = int(solver._iter_count)

    theta_deg = np.rad2deg(prob.get_val("cycle.structure.theta",units="rad",)).item()

    alpha_eff_deg = np.rad2deg(
        prob.get_val("cycle.aero.alpha_eff",units="rad",)).item()

    CL = prob.get_val("cycle.aero.CL").item()

    CD = prob.get_val("cycle.aero.CD").item()

    L_D = prob.get_val("performance.L_D").item()

    prob.cleanup()

    reader = om.CaseReader(str(recorder_path))

    case_ids = reader.list_cases("root.cycle.nonlinear_solver",recurse=False,out_stream=None,)

    abs_errors = []
    rel_errors = []

    for case_id in case_ids:
        case = reader.get_case(case_id)
        abs_errors.append(float(case.abs_err))
        rel_errors.append(float(case.rel_err))

    return {
        "kind": kind,
        "iterations": iteration_count,
        "abs_errors": abs_errors,
        "rel_errors": rel_errors,
        "theta_deg": theta_deg,
        "alpha_eff_deg": alpha_eff_deg,
        "CL": CL,
        "CD": CD,
        "L_D": L_D,
    }


def write_iteration_log(result):
    filename = (OUTPUT_DIR/ f"iterations_{result['kind']}.csv")

    with open(filename,"w",newline="",encoding="utf-8",) as stream:

        writer = csv.writer(stream)

        writer.writerow(["iteration","absolute_residual","relative_residual",])

        for i, (abs_err, rel_err) in enumerate(
            zip(result["abs_errors"],result["rel_errors"],)):
            writer.writerow([i, abs_err, rel_err,])


def plot_convergence(results):
    plt.figure(figsize=(7, 5))

    for result in results:

        residuals = np.maximum(np.asarray(result["abs_errors"]),1.0e-16,)

        iterations = np.arange(len(residuals))

        plt.semilogy(
            iterations,
            residuals,
            marker="o",
            label=(f"{SOLVER_LABELS[result['kind']]} "f"({result['iterations']} iterations)"),
        )

    plt.xlabel("Solver iteration")
    plt.ylabel("Residual norm")

    plt.title("Aeroelastic coupling convergence")

    plt.grid(True,which="both",alpha=0.3,)

    plt.legend()
    plt.tight_layout()

    plt.savefig(OUTPUT_DIR / "convergence.png",dpi=300)

    plt.close()

def plot_pair(results,kinds,filename,title):
    """
    Make a pairwise convergence plot.
    """
    selected = [result
        for result in results
        if result["kind"] in kinds]

    plt.figure(figsize=(7, 5))

    for result in selected:

        residuals = np.maximum(np.asarray(result["abs_errors"]),1.0e-16)

        iterations = np.arange(len(residuals))

        plt.semilogy(
            iterations,
            residuals,
            marker="o",
            label=(f"{SOLVER_LABELS[result['kind']]} "f"({result['iterations']} iterations)"))

    plt.xlabel("Solver iteration")

    plt.ylabel("Residual norm")

    plt.title(title)
    plt.grid(True,which="both",alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(OUTPUT_DIR / filename,dpi=300)

    plt.close()

def write_summary(results):
    summary_path = (OUTPUT_DIR / "solver_summary.txt")

    with open(summary_path,"w",encoding="utf-8",) as stream:

        stream.write("Aeroelastic Solver Comparison\n")

        stream.write("==============================\n\n")

        stream.write("Tolerance settings:\n")

        stream.write("atol = 1e-10\n")

        stream.write("rtol = 1e-10\n")

        stream.write("maxiter = 50\n\n")

        for result in results:

            stream.write(f"{SOLVER_LABELS[result['kind']]}\n")

            stream.write(f"Iterations: "f"{result['iterations']}\n")

            stream.write(f"theta: "f"{result['theta_deg']:.8f} deg\n")

            stream.write(f"alpha_eff: "f"{result['alpha_eff_deg']:.8f} deg\n")

            stream.write(f"CL: "f"{result['CL']:.8f}\n")

            stream.write(f"CD: "f"{result['CD']:.8f}\n")

            stream.write( f"L/D: "f"{result['L_D']:.8f}\n\n")


def main():
    OUTPUT_DIR.mkdir(exist_ok=True)

    with tempfile.TemporaryDirectory() as tmp:

        work_dir = Path(tmp)

        results = [run_solver("nlbgs", work_dir,),run_solver("nlbgs_aitken",work_dir,),run_solver("newton",work_dir,),run_solver("broyden",work_dir,)]

    for result in results:
        write_iteration_log(result)

    plot_convergence(results)

    # NLBGS vs Newton
    plot_pair(
        results,
        kinds=("nlbgs","newton",),
        filename=("convergence_nlbgs_vs_newton.png"),
        title=("NLBGS vs Newton Convergence"),
    )

    # Newton vs Broyden
    plot_pair(
        results,
        kinds=("newton","broyden",),
        filename=("convergence_newton_vs_broyden.png"),
        title=("Newton vs Broyden Convergence"),
    )

    # NLBGS vs NLBGS + Aitken
    plot_pair(
        results,
        kinds=("nlbgs","nlbgs_aitken"),
        filename=("convergence_nlbgs_vs_aitken.png"),
        title=("NLBGS vs NLBGS + Aitken Convergence"),
    )


    write_summary(results)

    print("\nSOLVER COMPARISON")
    print("--------------------------------")

    for result in results:
        print(
            f"{SOLVER_LABELS[result['kind']]:18s} "
            f"| iterations = "
            f"{result['iterations']:3d} "
            f"| theta = "
            f"{result['theta_deg']:.6f} deg "
            f"| CL = "
            f"{result['CL']:.6f}"
        )

    #print("\nGenerated:")
    #print("  outputs/convergence.png")
    #print("  outputs/iterations_nlbgs.csv")
    #print("  outputs/""iterations_nlbgs_aitken.csv")
    #print("  outputs/solver_summary.txt")


if __name__ == "__main__":
    main()