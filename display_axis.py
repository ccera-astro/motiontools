#!/usr/bin/env python3
# importing required librarie
import sys
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtWidgets import QVBoxLayout, QLabel
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QTimer, QTime, Qt
import argparse
import xmlrpc.client as xmlrpc

rpchandle = None
inited = False
def get_level(xmlstring):
    global rpchandle
    global inited
    r = 0.0
    try:
        if (inited == False):
            rpchandle = xmlrpc.ServerProxy(xmlstring, allow_none=True)
        r = rpchandle.get_level()
    except:
        r = -99.99
        pass
    return r


class Window(QWidget):

    def __init__(self, axis="Default Title", infile="/dev/null", ftsize=50, xmlrpc=None):
        super().__init__()

        # setting geometry of main window
        self.setGeometry(100, 100, 550, 200)
        
        self.axis = axis
        self.infile = infile
        self.xmlrpc = xmlrpc

        # creating a vertical layout
        layout = QVBoxLayout()

        # creating font object
        font = QFont('Monospace', ftsize, QFont.Bold)

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

        if (self.xmlrpc == None):
            fp = open(self.infile, "r")
            line = fp.readline()
            fp.close()
            
            if (len(line) >= 5):
                line = line.strip("\n")
                # showing it to the label
                self.label.setText(self.axis+": "+line)
        else:
            l = get_level(self.xmlrpc)
            self.label.setText(self.axis+": %6.2f" % l)


parser = argparse.ArgumentParser(description="Display axis position")

parser.add_argument("--axis", type=str, help="Axis Label", default="Elev")
parser.add_argument("--infile", type=str, help="Input file", default="/dev/null")
parser.add_argument("--fontsize", type=int, help="Font Size", default=50)
parser.add_argument("--xmlrpc", type=str, help="XMLRPC server", default=None)

args = parser.parse_args()

# create pyqt5 app
App = QApplication(sys.argv)

# create the instance of our Window
window = Window(axis=args.axis, infile=args.infile, ftsize=args.fontsize, xmlrpc=args.xmlrpc)

# showing all the widgets
window.show()

# start the app
App.exit(App.exec_())
