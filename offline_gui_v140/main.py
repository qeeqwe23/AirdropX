from __future__ import annotations

from pathlib import Path
import sys

try:
    from PyQt6.QtWidgets import QApplication
except ImportError:
    print("缺少 PyQt6。请先安装 requirements.txt，或使用打包后的 EXE。")
    raise

from ui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("AirdropX Offline GUI v1.4.0")
    root = Path(__file__).resolve().parent
    win = MainWindow(root)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
