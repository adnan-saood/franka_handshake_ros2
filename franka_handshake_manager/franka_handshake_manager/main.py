import sys
from PyQt5.QtWidgets import QApplication
from franka_handshake_manager.qt_main_window import MainWindow

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
