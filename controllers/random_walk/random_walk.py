# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container with Random Walk."""
from controller import Supervisor

import optparse
import math
import random  # Importado para gerar os valores aleatórios


class Pedestrian (Supervisor):
    """Control a Pedestrian PROTO with Random Walk."""

    def __init__(self):
        """Constructor: initialize constants."""
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.50
        self.speed = 1.15
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        Supervisor.__init__(self)

    def run(self):
        """Set the Pedestrian pose and position dynamically."""
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--trajectory", default="", help="Ignored in random walk")
        opt_parser.add_option("--speed", type=float, default=0.5, help="Specify walking speed in [m/s]")
        opt_parser.add_option("--step", type=int, help="Specify time step (otherwise world time step is used)")
        options, args = opt_parser.parse_args()

        if options.speed and options.speed > 0:
            self.speed = options.speed
        if options.step and options.step > 0:
            self.time_step = options.step
        else:
            self.time_step = int(self.getBasicTimeStep())

        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField("translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.root_node_ref.getField(self.joint_names[i]))

        # Posição inicial baseada em onde o objeto foi arrastado no mapa
        current_pos = self.root_translation_field.getSFVec3f()
        current_x = current_pos[0]
        current_y = current_pos[1]

        # Centro e metade do quadrado 10x10 que confina o pedestre ao spawn
        origin_x = current_x
        origin_y = current_y
        half_box = 5.0

        # Configurações do Random Walk (Ajuste aqui se quiser passos maiores ou menores)
        step_range = 2.0  # Raio máximo em metros para adicionar/remover do passo atual

        # Sorteia um alvo relativo ao ponto atual, preso dentro do quadrado 10x10.
        def pick_target():
            tx = current_x + random.uniform(-step_range, step_range)
            ty = current_y + random.uniform(-step_range, step_range)
            tx = min(max(tx, origin_x - half_box), origin_x + half_box)
            ty = min(max(ty, origin_y - half_box), origin_y + half_box)
            return tx, ty

        # Gerar o primeiro ponto alvo aleatório
        target_x, target_y = pick_target()
        
        # Distância acumulada percorrida apenas para controlar a animação das pernas
        distance_walked = 0.0

        while not self.step(self.time_step) == -1:
            # Calcular vetor e distância até o alvo atual
            dx = target_x - current_x
            dy = target_y - current_y
            distance_to_target = math.sqrt(dx * dx + dy * dy)

            # Se chegou perto do alvo (menos de 10cm), escolhe uma nova posição aleatória
            if distance_to_target < 0.1:
                target_x, target_y = pick_target()
                continue

            # Passo linear a ser dado neste frame de tempo
            dt = self.time_step / 1000.0  # Converte milissegundos para segundos
            step_size = self.speed * dt

            # Garante que o passo não ultrapasse o alvo
            if step_size > distance_to_target:
                step_size = distance_to_target

            # Atualiza as posições X e Y avançando em direção ao alvo
            current_x += (dx / distance_to_target) * step_size
            current_y += (dy / distance_to_target) * step_size
            distance_walked += step_size

            # Controle da animação dos membros (baseado na distância percorrida)
            current_sequence = int((distance_walked / self.CYCLE_TO_DISTANCE_RATIO) % self.WALK_SEQUENCES_NUMBER)
            ratio = (distance_walked / self.CYCLE_TO_DISTANCE_RATIO) - int(distance_walked / self.CYCLE_TO_DISTANCE_RATIO)

            for i in range(0, self.BODY_PARTS_NUMBER):
                current_angle = self.angles[i][current_sequence] * (1 - ratio) + \
                    self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
                self.joints_position_field[i].setSFFloat(current_angle)

            # Ajuste de oscilação da altura (quadril)
            self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + \
                self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

            # Aplica a rotação olhando para o alvo atual
            angle = math.atan2(dy, dx)
            rotation = [0, 0, 1, angle]

            # Envia as novas coordenadas para o Webots
            root_translation = [current_x, current_y, self.ROOT_HEIGHT + self.current_height_offset]
            self.root_translation_field.setSFVec3f(root_translation)
            self.root_rotation_field.setSFRotation(rotation)


controller = Pedestrian()
controller.run()
