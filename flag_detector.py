

class FlagDetector:
    """Detects False to True transitions on an external signal."""

    def __init__(self, reader, action):
        self.reader = reader
        self.action = action
        self.last_value = reader()    # initialise value

    def test(self):
        new_value = self.reader()
        if new_value and not self.last_value:
            self.action()
        self.last_value = new_value