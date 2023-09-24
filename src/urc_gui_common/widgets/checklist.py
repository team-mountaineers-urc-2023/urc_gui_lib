import os
import yaml
from typing import Dict, List, Generator, Union

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from urc_gui_common.ui_python.checklist_widget import Ui_Checklist
from urc_gui_common.helpers.file_helper import file_basenames

mission_checklists_dir = os.path.expanduser('~/.ros/mission_checklists')

class ChecklistWidget(QWidget):
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		self.ui = Ui_Checklist()
		self.ui.setupUi(self) 

		self.ui.list_selection.addItems(file_basenames(mission_checklists_dir))
		self.ui.list_selection.currentTextChanged.connect(self.load_checklist)

		self.ui.expand_button.clicked.connect(self.expand_checklist)
		self.ui.collapse_button.clicked.connect(self.collapse_checklist)

	def load_checklist(self):
		self.ui.tree.clear()

		# make sure the user actually selected a checklist
		file_basename = self.ui.list_selection.currentText()
		if file_basename == "":
			return

		# make sure the checklist file exists
		filepath = f'{mission_checklists_dir}/{file_basename}.yaml'
		if not os.path.exists(filepath):
			QMessageBox(
				QMessageBox.Critical,
				"Checklist does not exist",
				f"Checklist at {filepath} does not exist",
				QMessageBox.Ok
			).exec()
			return

		# load the checklist contents
		with open(filepath, 'r') as checklist_file:
			checklist = yaml.safe_load(checklist_file)
			tasks = self.make_tasks_from_items(checklist)
			for task in tasks:
				self.ui.tree.addTopLevelItem(task)
			self.ui.tree.expandAll()
			self.ui.tree.setSelectionMode(QAbstractItemView.NoSelection)
			self.ui.tree.setFocusPolicy(Qt.NoFocus)

	def make_tasks_from_items(self, items: List[Union[str, Dict]]) -> Generator[QTreeWidgetItem, None, None]:
		for item in items:
			for task in self.make_tasks_from_item(item):
				yield task

	def make_tasks_from_item(self, item: Union[str, Dict[str, List]]) -> Generator[QTreeWidgetItem, None, None]:
		def make_tree_widget_item(text):
			item = QTreeWidgetItem((text,))
			item.setCheckState(0, Qt.Unchecked)
			item.setSizeHint(0, QSize(-1, 25))
			return item

		if type(item) == dict:
			for k, v in item.items():
				task = make_tree_widget_item(k)
				child_tasks = self.make_tasks_from_items(v)
				task.addChildren(child_tasks)
				yield task
		else:
			task = make_tree_widget_item(item)
			yield task

	def expand_checklist(self):
		self.ui.tree.expandAll()

	def collapse_checklist(self):
		self.ui.tree.collapseAll()
