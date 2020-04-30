import yaml
import numpy as np

map_rois = {'rois': []}

with open('world14_plants.yaml') as f:
    
    plant_locs = yaml.load(f, Loader=yaml.FullLoader)
    
    for plant in plant_locs:
        model = plant['model']
        location = np.asarray(plant['location'])
        with open (model + '_rois.yaml') as p:
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

with open('gt_w14.yaml', 'w') as f:
    data = yaml.dump(map_rois, f, default_flow_style=None)

