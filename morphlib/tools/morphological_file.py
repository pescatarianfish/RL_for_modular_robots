from morphlib.tools.template_renderer import TemplateRenderer
import morphlib.tools.utils as u
import numpy as np
import os

class MorphologicalFile(object):

    def __init__(self, filepath) -> None:
        
        self.renderer = TemplateRenderer(filepath)
        self.default_parameters = u.get_default_values(filepath)
        self.parameters = self.default_parameters
        self.morph_params = self.parameters['morph_params']
        self.morph_limits = self.parameters['morph_limits']
        self.morph_groups = {}
        self.user_mp_map = {}

        # self.build_morphological_file() # sets self.morph_file
    
    def get_morphological_file(self, morph_params=None, asarray=False):

        if np.all(morph_params != None):
            self.update_morph_params(morph_params, asarray=asarray)

        self.morph_file = self.renderer.render_template(**self.parameters)

        return self.morph_file
    
    def build_morphological_file(self, morph_file_path, morph_params=None, asarray=False):

        if np.all(morph_params != None):
            self.update_morph_params(morph_params, asarray=asarray)
        
        self.morph_file = self.renderer.render_to_file(morph_file_path, **self.parameters)
    
    def get_param(self, name, asarray=False):

        if asarray and bool(self.user_mp_map):
            params = self.parameters[name]
            params_subset = {k: params[k] for k in self.user_mp_map.keys()}
            _, v = u.dict2nparray(params_subset)
            return v

        elif asarray:
            k, v = u.dict2nparray(self.parameters[name])
            return v
    
        elif bool(self.user_mp_map):
            params = self.parameters[name]
            for group_name, associated in self.morph_groups.items():
                params[group_name] = params[associated[0]]
            params_subset = {v: params[v] for v in self.user_mp_map.values()}
            return params_subset

        return self.parameters[name]

    def get_morph_params(self, asarray=False) -> list:
        '''
        :return: list of morphology parameters and there current value
        :rtype: list 
        '''
        return self.get_param('morph_params', asarray=asarray)

    def get_morph_limits(self, asarray=False) -> list:
        limits = self.get_param('morph_limits', asarray=asarray)

        lower = np.empty(len(limits))
        upper = np.empty(len(limits))
        for i,l in enumerate(limits):
            lower[i] = l[0]
            upper[i] = l[1]

        return np.array([lower, upper])

    def get_user_map_data(self, params):
        d = {}
        print(params)
        for idx in self.user_mp_map.keys():
            print(idx)
            d[self.user_mp_map[idx]] = params[idx]
        return d

    def update_params(self, name, params, asarray=False) -> list:
        if asarray and bool(self.user_mp_map):
            params = self.get_user_map_data(params)

        elif asarray:
            params = u.nparray2dict(list(self.parameters[name].keys()), params)

        params2change = params.keys()
        _params = self.parameters['morph_params']
        for group_name, mp_names in self.morph_groups.items():
            if group_name in params2change:
                val = params.pop(group_name)
                for param_name in mp_names:
                    _params[param_name] = val
        _params.update(params)
        self.parameters['morph_params'] = _params

    def update_morph_limits(self, limits, asarray=False) -> list:
        self.update_params('morph_limits', limits, asarray=asarray) 

    def update_morph_params(self, params, asarray=False):
        self.update_params('morph_params', params, asarray=asarray) 

    def get_morph_dims(self) -> int:
        '''
        :returns: how many morphological parameters there are
        :rtype: int
        '''
        return len(self.parameters['morph_params'])