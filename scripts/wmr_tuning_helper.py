#!/usr/bin/env python3
"""
wmr_tuning_helper.py

Small helper script to explore and print suggested parameter sets
for the WMR NSMC/BSMC controllers.

Usage examples:

  python3 scripts/wmr_tuning_helper.py
  python3 scripts/wmr_tuning_helper.py --profile conservative
  python3 scripts/wmr_tuning_helper.py --controller BSMC --trajectory figure8
"""

import argparse
import textwrap
from dataclasses import dataclass, asdict


@dataclass
class GainSet:
    name: str
    description: str
    controller_type: str
    trajectory: str
    R: float
    Omega: float
    Vmax: float
    Wmax: float
    a11: float
    a12: float
    a21: float
    a22: float
    k1: float
    k2: float
    eps: float
    use_disturbance: bool = False


def get_presets():
    """Return a dict of named presets."""
    presets = {}

    # 1) Conservative – pour éviter les gros dépassements, stable mais lent
    presets["conservative"] = GainSet(
        name="conservative",
        description="Small circle, low speed, soft gains. Good for first tests.",
        controller_type="NSMC",
        trajectory="circle",
        R=0.5,
        Omega=0.15,
        Vmax=0.18,
        Wmax=1.0,
        a11=5.0,
        a12=3.0,
        a21=3.0,
        a22=2.0,
        k1=4.0,
        k2=6.0,
        eps=1.5,
        use_disturbance=False,
    )

    # 2) Nominal – ce qu’on a utilisé dans la dernière version de ton contrôleur
    presets["nominal"] = GainSet(
        name="nominal",
        description="Balanced tracking vs smoothness. Good general setting.",
        controller_type="NSMC",
        trajectory="circle",
        R=0.6,
        Omega=0.20,
        Vmax=0.22,
        Wmax=1.8,
        a11=8.0,
        a12=5.0,
        a21=4.0,
        a22=3.0,
        k1=5.0,
        k2=7.0,
        eps=2.0,
        use_disturbance=False,
    )

    # 3) Aggressive – plus de tracking, plus de risque de chattering
    presets["aggressive"] = GainSet(
        name="aggressive",
        description="Stronger tracking, sharper turns. Use with care.",
        controller_type="NSMC",
        trajectory="circle",
        R=0.8,
        Omega=0.25,
        Vmax=0.25,
        Wmax=2.0,
        a11=12.0,
        a12=8.0,
        a21=7.0,
        a22=5.0,
        k1=7.0,
        k2=10.0,
        eps=3.0,
        use_disturbance=True,
    )

    # 4) Figure-8 profile – pour tester la lemniscate Gerono
    presets["figure8_nominal"] = GainSet(
        name="figure8_nominal",
        description="Nominal gains adapted to figure-8 trajectory.",
        controller_type="NSMC",
        trajectory="figure8",
        R=0.7,
        Omega=0.18,
        Vmax=0.22,
        Wmax=1.8,
        a11=8.0,
        a12=5.0,
        a21=4.0,
        a22=3.0,
        k1=5.0,
        k2=7.0,
        eps=2.0,
        use_disturbance=False,
    )

    # 5) BSMC nominal – même niveau que NSMC nominal mais avec BSMC
    presets["bsmc_nominal"] = GainSet(
        name="bsmc_nominal",
        description="BSMC variant with similar magnitudes as NSMC nominal.",
        controller_type="BSMC",
        trajectory="circle",
        R=0.6,
        Omega=0.20,
        Vmax=0.22,
        Wmax=1.8,
        a11=8.0,
        a12=5.0,
        a21=4.0,
        a22=3.0,
        k1=5.0,
        k2=7.0,
        eps=2.0,
        use_disturbance=False,
    )

    return presets


def print_gainset(gs: GainSet):
    """Nicely print parameters and give the matching ros2 run command."""
    print("=" * 70)
    print(f"Preset name     : {gs.name}")
    print(f"Description     : {gs.description}")
    print("-" * 70)
    print("Controller / trajectory parameters:")
    print(f"  controller_type : {gs.controller_type}")
    print(f"  trajectory      : {gs.trajectory}")
    print(f"  R               : {gs.R:.3f}")
    print(f"  Omega           : {gs.Omega:.3f}  # rad/s")
    print(f"  Vmax            : {gs.Vmax:.3f}   # m/s")
    print(f"  Wmax            : {gs.Wmax:.3f}   # rad/s")
    print()
    print("Gains (sliding surfaces & nonlinear terms):")
    print(f"  a11, a12        : {gs.a11:.3f}, {gs.a12:.3f}")
    print(f"  a21, a22        : {gs.a21:.3f}, {gs.a22:.3f}")
    print(f"  k1, k2          : {gs.k1:.3f}, {gs.k2:.3f}")
    print(f"  eps (tanh slope): {gs.eps:.3f}")
    print(f"  use_disturbance : {gs.use_disturbance}")
    print("-" * 70)
    print("ROS2 command example:")
    print()

    cmd = [
        "ros2 run wmr_controller wmr_controller",
        "--ros-args",
        f"-p controller_type:={gs.controller_type}",
        f"-p trajectory:={gs.trajectory}",
        f"-p R:={gs.R}",
        f"-p Omega:={gs.Omega}",
        f"-p Vmax:={gs.Vmax}",
        f"-p Wmax:={gs.Wmax}",
        f"-p use_disturbance:={'true' if gs.use_disturbance else 'false'}",
    ]

    # On affiche sur plusieurs lignes pour que ce soit lisible
    print("  " + " \\\n  ".join(cmd))
    print("=" * 70)
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Helper to inspect and generate WMR SMC/BSMC tuning presets.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            Examples:
              python3 scripts/wmr_tuning_helper.py
              python3 scripts/wmr_tuning_helper.py --profile conservative
              python3 scripts/wmr_tuning_helper.py --profile figure8_nominal
              python3 scripts/wmr_tuning_helper.py --profile bsmc_nominal
            """
        ),
    )

    parser.add_argument(
        "--profile",
        type=str,
        default=None,
        help="Preset name to display (conservative, nominal, aggressive, "
             "figure8_nominal, bsmc_nominal). If omitted, all are printed.",
    )

    args = parser.parse_args()
    presets = get_presets()

    if args.profile is None:
        print("Available presets:\n")
        for name in presets:
            print_gainset(presets[name])
    else:
        key = args.profile.strip().lower()
        if key not in presets:
            print(f"ERROR: profile '{args.profile}' not found.")
            print("Available profiles:", ", ".join(presets.keys()))
            return
        print_gainset(presets[key])


if __name__ == "__main__":
    main()