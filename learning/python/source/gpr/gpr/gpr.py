import pickle

import numpy as np


class GPR(object):

    def __init__(self, filename):
        try:
            with open(filename, 'rb') as f:
                model = pickle.load(f)
            self.gpr = model['gpr']
            self.scaler = model['scaler']
            del model
        except IOError as e:
            print(e)
            print("Choose a valid file and try again!")
            exit(0)

        self.input_dim = self.gpr.X_train_.shape[1]

    def predict(self, state):
        state = np.array(state).reshape(1, -1)
        if state.shape[1] != self.input_dim:
            print("The length of the input is not correct:", state.shape[1], "!=", self.input_dim)
            exit(0)
        pred_force, std = self.gpr.predict(state, return_std=True)
        pred_force = self.scaler.inverse_transform(pred_force)
        std = std * self.scaler.scale_
        return [pred_force, std]
