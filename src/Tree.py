

class Node(object):
	def __init__(self, value):
		self.value=value
		self.children: List[Node] = []

	def print(self):
		self._printRec(0)

	def _printRec(self, padding:int):
		paddingStr = ""
		for _ in range(padding):
			paddingStr+='\t'

		print(f"{paddingStr}{self.value}")
		for child in self.children:
			child._printRec(padding+1)

class Tree(object):
	def __init__(self, root: Node = None):
		self.root = root

	def print(self):
		self.root.print()
