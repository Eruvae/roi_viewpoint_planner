import yaml
import numpy as np
import sys

if len(sys.argv) < 2:
    print('Error: world name not specified')
    quit()

world_name = sys.argv[1]

map_rois = {'rois': []}

with open('world_plant_locations/' + world_name + '_plants.yaml') as f:
    
    plant_locs = yaml.load(f, Loader=yaml.FullLoader)
    
    for plant in plant_locs:
        model = plant['model']
        location = np.asarray(plant['location'])
        with open ('plant_files/' + model + '_rois.yaml') as p:
            roi_locs = yaml.load(p, Loader=yaml.FullLoader)
            for roi in roi_locs:
                minp = np.asarray(roi['min'])
                maxp = np.asarray(roi['max'])
                center = location + (minp + maxp) / 2.0
                size = maxp - minp
                map_rois['rois'].append({'location': center.tolist(), 'size': size.tolist()})

def float_representer(dumper, value):
    text = '{0:.4f}'.format(value)
    return dumper.represent_scalar(u'tag:yaml.org,2002:float', text)
yaml.add_representer(float, float_representer)

with open('world_roi_gts/' + world_name + '_roi_gt.yaml', 'w') as f:
    data = yaml.dump(map_rois, f, default_flow_style=None)

