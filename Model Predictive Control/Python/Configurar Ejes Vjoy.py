import pyvjoy
import numpy as np


class VJoyOutput:
    def __init__(self, device_id=1):
        self.j = pyvjoy.VJoyDevice(device_id)
        self.center()

    def send_all(self, steer_rad, u_cmd, max_rad=1.2):
        """
        steer_rad:
            Dirección en radianes [-1.2, +1.2]

        u_cmd:
            Comando longitudinal [-1, +1]
            +1 = acelerador
             0 = sin acelerador/freno
            -1 = freno
        """

        # -------------------------
        # DIRECCIÓN -> EJE X
        # -------------------------
        norm = np.clip(steer_rad / max_rad, -1.0, 1.0)

        steer_val = int(
            (norm + 1.0) / 2.0 * 32766 + 1
        )

        steer_val = max(1, min(32767, steer_val))

        # -------------------------
        # ACELERADOR / FRENO
        # -------------------------
        if u_cmd >= 0:
            gas_val = int(u_cmd * 32767)
            brake_val = 0
        else:
            gas_val = 0
            brake_val = int(-u_cmd * 32767)

        # -------------------------
        # ENVIAR A VJOY
        # -------------------------
        self.j.data.wAxisX = steer_val
        self.j.data.wAxisY = gas_val
        self.j.data.wAxisZRot = brake_val

        self.j.update()

        return (
            gas_val / 32767.0,
            brake_val / 32767.0
        )

    def center(self):
        # Volante al centro
        self.j.data.wAxisX = 16384

        # Acelerador 0
        self.j.data.wAxisY = 0

        # Freno 0
        self.j.data.wAxisZRot = 0

        self.j.update()