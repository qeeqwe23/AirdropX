from __future__ import annotations

from pathlib import Path
import sys

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    print("缺少 PyQt6。请先运行 install_python_env.ps1，或使用安装脚本自动构建。")
    raise

from ui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("AirdropX Offline Software v1.0")
    root = Path(__file__).resolve().parent
    win = MainWindow(root)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())