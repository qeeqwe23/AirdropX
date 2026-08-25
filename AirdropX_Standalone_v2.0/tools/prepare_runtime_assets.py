from __future__ import annotations
import argparse, shutil
from pathlib import Path

from export_controller_bank import discover_bank, export_bank


def copy_tree(src: Path, dst: Path) -> None:
    if not src.is_dir():
        raise FileNotFoundError(f"Required JSBSim asset folder not found: {src}")
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


def main() -> int:
    ap=argparse.ArgumentParser(description='Prepare all assets needed by standalone AirdropX runtime.')
    ap.add_argument('--project-root', type=Path, required=True)
    ap.add_argument('--app-root', type=Path, required=True)
    ns=ap.parse_args()
    project=ns.project_root.resolve(); app=ns.app_root.resolve()
    assets=app/'assets'; controller=assets/'controller'; jsb=assets/'jsbsim'
    controller.mkdir(parents=True,exist_ok=True); jsb.mkdir(parents=True,exist_ok=True)
    info=discover_bank(project)
    export_bank(info.path, controller/'controller_bank.npz')
    # Only ship the aircraft actually used by AirdropX; keep the full engine folder because
    # MQ9_Reaper may reference engine/thruster XMLs by name.
    (jsb/'aircraft').mkdir(parents=True,exist_ok=True)
    copy_tree(project/'aircraft'/'MQ9_Reaper', jsb/'aircraft'/'MQ9_Reaper')
    copy_tree(project/'engine', jsb/'engine')
    systems=project/'systems'
    if systems.is_dir(): copy_tree(systems, jsb/'systems')
    else: (jsb/'systems').mkdir(exist_ok=True)
    print('ASSET_PREP_PASS')
    print(f'controller_bank={controller/"controller_bank.npz"}')
    print(f'jsbsim_assets={jsb}')
    return 0
if __name__=='__main__': raise SystemExit(main())
