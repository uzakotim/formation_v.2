# import pyyaml module
import yaml
from yaml.loader import SafeLoader

# Open the file and load the file
with open('./custom_configs/uav_names.yaml') as f:
    data = yaml.load(f, Loader=SafeLoader)
for i in data['network']['robot_names']:
    print(i[3:])