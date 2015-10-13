from PyQt4 import QtCore, QtGui


class MainWindow(QtGui.QMainWindow):
    def __init__(self, *args, **kwargs):
        QtGui.QMainWindow.__init__(self, *args, **kwargs)
        self.keysPressed = []
        self.keyHits = []
        self.mouseClicks = []

    def mousePressEvent(self, ev):
        # print ev.button()
        # print self.view.ItemTransformOriginPointChange
        # print ev.x(), ev.y(), ev.pos()
        ev.accept()
        self.mouseClicks.append([ev.x(), ev.y(), ev.pos(), ev.button()])

    def keyPressEvent(self, ev):
        print ev.key(), QtCore.Qt.Key_Up
        ev.accept()
        if ev.key() not in self.keyHits:
            self.keyHits.append(ev.key())
        if ev.isAutoRepeat():
            return
        self.keysPressed.append(ev.key())

    def keyReleaseEvent(self, ev):
        # ev.accept()
        if ev.isAutoRepeat():
            return
        if ev.key() in self.keysPressed:
            self.keysPressed.remove(ev.key())
        else:
            print "key hit of", ev.key(), "not detected"