
class TestM:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        self.axes = config.get('axes', 'XYZ').upper()
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
#        self.template = gcode_macro.load_template(config, 'gcode')
        self.in_script = False
        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
		#register a traditional command, i.e. one that does not expect parameters with equal sign. If we registered a modern command
		#the gcode handler (prev_G28) would not know how to handle the parameters, and we would need to build the command anew, with all 
		#the error handling.

    def huihai(self, gcmd):
        
            toolhead = self.printer.lookup_object('toolhead')
            
def load_config(config):
    return TestM(config)