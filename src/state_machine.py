from PyQt4.QtCore import QObject, pyqtSignal

class Foo(QObject):

    # Define a new signal called 'trigger' that has no arguments.
    trigger = pyqtSignal()
    button_click = pyqtSignal()


    def connect_and_emit_trigger(self):
        # Connect the trigger signal to a slot.
        self.trigger.connect(self.handle_trigger)
        self.button_click.connect(self.pushed)

        # # Emit the signal.
        self.trigger.emit()
        self.button_click.emit()
        # self.trigger.emit()
        # self.trigger.emit()

    def handle_trigger(self):
        # Show that the slot has been called.

        print "trigger signal received"

    def pushed(self):
        print "You're really pushing my buttons"

f = Foo()

f.connect_and_emit_trigger()
