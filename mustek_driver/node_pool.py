import threading


class NodePool:
    def __init__(self, block_size: int, node_size: int):
        self.block_size = block_size
        self.node_size = node_size
        self._free = []
        self._lock = threading.Lock()

    def push_node(self, node):
        if node is None:
            return
        with self._lock:
            self._free.append(node)

    def push_node_front(self, node):
        """Insert at the head of the free list. Combined with the tail-pop of
        pop_node this yields FIFO dequeue order (first pushed pops first),
        which ScannerReflective.store_line uses for its scan-data ring."""
        if node is None:
            return
        with self._lock:
            self._free.insert(0, node)

    def pop_node(self):
        with self._lock:
            return self._free.pop() if self._free else None

    def drain(self):
        with self._lock:
            self._free.clear()
