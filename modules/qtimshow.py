#
#  This file is part of qtimshow
#  https://github.com/varhub/qtimshow.git
#
#  Copyright (C) 2015 Victor Arribas
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Victor Arribas <v.arribas.urjc@gmail.com>
#


"""
Little hack to have an imshow function like OpenCV
"""

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QImage
import numpy as np


__Qimshow = None


def enable():
    """
    Enables imshow.
    It must be runned from main thread.
    """
    global __Qimshow
    if __Qimshow is None:
        __Qimshow = Qimshow()
        print '''
qtimshow - Copyright (2015) Victor Arribas
  module enabled, enjoy it with:
  from qtimshow import imshow 
'''


def disable():
    global __Qimshow
    __Qimshow = None


def imshow(title, img, format=None):
    """
    :param title: Behaves like OpenCV imshow, BUT requires RGB isntead.
    :param img: RGB or grayscale image
    :param format: required if image is other than above
    """
    if __Qimshow is not None:
        if format is None:
            """ if None, set it is mandatory. PyQt new-style signals do not allow
            None value as wildcard, so SLOT will receive a undefined value """
            if len(img.shape) == 2:
                format = QImage.Format_Indexed8
            else:
                format = QImage.Format_RGB888

        __Qimshow.Q_SIGNAL_imshow.emit(img, title, format)


class Qimshow(QtCore.QObject):
    """ Only objects (that inherit from Qobjects) can define PyQt new-style signals.
    This class wraps Qt requirements for message passing.
    """

    Q_SIGNAL_imshow = QtCore.pyqtSignal(np.ndarray, str, int)

    def __init__(self):
        QtCore.QObject.__init__(self)
        self.Q_SIGNAL_imshow.connect(self._qtimshow)

    @QtCore.pyqtSlot(np.ndarray, str, int)
    def _qtimshow(self, img, title, format):
        if len(img.shape) == 2:
            qlen = img.shape[1]
        else:
            qlen = img.shape[1]*img.shape[2]
        qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], qlen, format);

        WinSize = QtCore.QSize(img.shape[1], img.shape[0])

        qui_win = self._fetchWin(title)
        if qui_win is None:
            qui_win = QtGui.QWidget()
            qui_win.setWindowTitle(title)
            qui_win.resize(WinSize)

            qui_imgLabel = QtGui.QLabel(qui_win)
            qui_imgLabel.move(0,0);
            qui_imgLabel.resize(WinSize)
            qui_imgLabel.show()

            qui_win.setVisible(True)

            self._addWin(qui_win)

        qui_imgLabel = qui_win.children()[0]
        qui_imgLabel.setPixmap(QtGui.QPixmap.fromImage(qimg))

        return qui_win


    __win_list = []

    def _addWin(self, win):
        if win not in self.__win_list:
            self.__win_list.append(win)

    def _fetchWin(self, title):
        for win in self.__win_list:
            if title == win.windowTitle():
                return win
        return None
