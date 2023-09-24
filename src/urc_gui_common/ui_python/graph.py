# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_xml/graph.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_GraphWidget(object):
    def setupUi(self, GraphWidget):
        GraphWidget.setObjectName("GraphWidget")
        GraphWidget.resize(400, 300)
        self.gridLayout = QtWidgets.QGridLayout(GraphWidget)
        self.gridLayout.setObjectName("gridLayout")
        self.graph = PlotWidget(GraphWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.graph.sizePolicy().hasHeightForWidth())
        self.graph.setSizePolicy(sizePolicy)
        self.graph.setObjectName("graph")
        self.gridLayout.addWidget(self.graph, 0, 0, 1, 3)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.autoscale_button = QtWidgets.QPushButton(GraphWidget)
        self.autoscale_button.setObjectName("autoscale_button")
        self.horizontalLayout.addWidget(self.autoscale_button)
        self.default_scale_button = QtWidgets.QPushButton(GraphWidget)
        self.default_scale_button.setObjectName("default_scale_button")
        self.horizontalLayout.addWidget(self.default_scale_button)
        self.gridLayout.addLayout(self.horizontalLayout, 2, 0, 1, 3)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.x_label = QtWidgets.QLabel(GraphWidget)
        self.x_label.setMinimumSize(QtCore.QSize(0, 0))
        self.x_label.setObjectName("x_label")
        self.horizontalLayout_2.addWidget(self.x_label)
        self.y_label = QtWidgets.QLabel(GraphWidget)
        self.y_label.setMinimumSize(QtCore.QSize(0, 0))
        self.y_label.setObjectName("y_label")
        self.horizontalLayout_2.addWidget(self.y_label, 0, QtCore.Qt.AlignBottom)
        self.gridLayout.addLayout(self.horizontalLayout_2, 1, 2, 1, 1)

        self.retranslateUi(GraphWidget)
        QtCore.QMetaObject.connectSlotsByName(GraphWidget)

    def retranslateUi(self, GraphWidget):
        _translate = QtCore.QCoreApplication.translate
        GraphWidget.setWindowTitle(_translate("GraphWidget", "GraphWidget"))
        self.autoscale_button.setText(_translate("GraphWidget", "Autoscale"))
        self.default_scale_button.setText(_translate("GraphWidget", "Default Scale"))
        self.x_label.setText(_translate("GraphWidget", "x: -"))
        self.y_label.setText(_translate("GraphWidget", "y: -"))
from pyqtgraph import PlotWidget