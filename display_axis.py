#!/usr/bin/env python3
# importing required librarie
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtWidgets import QVBoxLayout, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QTimer, QTime, Qt
import argparse


class Window(QWidget):

    def __init__(self, axis="Default Title", infile="/dev/null"):
        super().__init__()

        # setting geometry of main window
        self.setGeometry(100, 100, 550, 300)
        
        self.axis = axis
        self.infile = infile

        # creating a vertical layout
        layout = QVBoxLayout()

        # creating font object
        font = QFont('Monospace', 50, QFont.Bold)

        # creating a label object
        self.label = QLabel()

        # setting centre alignment to the label
        self.label.setAlignment(Qt.AlignCenter)

        # setting font to the label
        self.label.setFont(font)

        # adding label to the layout
        layout.addWidget(self.label)

        # setting the layout to main window
        self.setLayout(layout)

        # creating a timer object
        timer = QTimer(self)

        # adding action to timer
        timer.timeout.connect(self.showData)

        # update the timer every second
        timer.start(200)

    # method called by timer
    def showData(self):

        fp = open(self.infile, "r")
        line = fp.readline()
        fp.close()
        
        if (len(line) >= 5):
            line = line.strip("\n")
            # showing it to the label
            self.label.setText(self.axis+": "+line)


parser = argparse.ArgumentParser(description="Display axis position")

parser.add_argument("--axis", type=str, help="Axis Label", default="Elev")
parser.add_argument("--infile", type=str, help="Input file", default="/dev/null")

args = parser.parse_args()

# create pyqt5 app
App = QApplication(sys.argv)

# create the instance of our Window
window = Window(axis=args.axis, infile=args.infile)

# showing all the widgets
window.show()

# start the app
App.exit(App.exec_())
