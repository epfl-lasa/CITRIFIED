import pickle

import numpy as np


class GPR(object):

    def __init__(self):
        self._ready = False
        self.input_dim = 0
        self._gpr, self._scaler = [], []

    def init(self, filename):
        print("[GPR::init] Loading file " + filename)
        try:
            with open(filename, 'rb') as f:
                model = pickle.load(f)
            self._gpr = model['gpr']
            self._scaler = model['scaler']
            del model
            self._ready = True
        except IOError as e:
            print(e)
            print("[GPR::init] Choose a valid file and try again!")

        self.input_dim = self._gpr.X_train_.shape[1]
        print("[GPR::init] GPR initialized")
        return self._ready

    def initialized(self):
        return self._ready

    def reset(self):
        print("[GPR::reset] Resetting GPR")
        self._ready = False
        self._gpr = []
        self._scaler = []

    def predict(self, state):
        if self._ready:
            state = np.array(state).reshape(1, -1)
            if state.shape[1] != self.input_dim:
                print("[GPR::predict] The length of the input is not correct:", state.shape[1], "!=", self.input_dim)
                exit(0)
            pred_force, std = self._gpr.predict(state, return_std=True)
            pred_force = self._scaler.inverse_transform(pred_force[0])
            std = std * self._scaler.scale_
            return [pred_force, std]
        else:
            return False
