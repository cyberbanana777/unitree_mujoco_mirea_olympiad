import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

# Добавляем класс для управления кнопками и баннером
class ButtonBannerController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.button_states = [False] * 5
        self.banner_visible = False
        self.current_button = -1  # Какая кнопка сейчас активна
        
        # Получаем ID кнопок и баннера
        self.button_joint_ids = []
        self.banner_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "banner_geom")
        
        # Получаем ID материалов для разных кнопок
        self.banner_material_ids = []
        for i in range(1, 6):
            # ID joints для кнопок
            joint_name = f"button{i}_joint"
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id != -1:
                self.button_joint_ids.append(joint_id)
            
            # ID материалов для баннера
            material_name = f"banner_material{i}"
            material_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_MATERIAL, material_name)
            self.banner_material_ids.append(material_id)
        
        # ID материала по умолчанию (скрытый)
        self.default_material_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_MATERIAL, "banner_material_default")
        
        # print(f"Found {len(self.button_joint_ids)} buttons")
        # print(f"Banner geom ID: {self.banner_geom_id}")
        # print(f"Material IDs: {self.banner_material_ids}")
        # print(f"Default material ID: {self.default_material_id}")
        
        # Изначально скрываем баннер
        self.set_banner_visibility(False)
    
    def update(self):
        """Обновляет состояние кнопок и баннера"""
        any_button_pressed = False
        pressed_button_index = -1
        
        # Проверяем все кнопки
        for i, joint_id in enumerate(self.button_joint_ids):
            # Получаем позицию кнопки (slide joint)
            button_pos = self.data.joint(joint_id).qpos[0]
            is_pressed = button_pos > -0.025  # Порог нажатия
            
            if is_pressed and not self.button_states[i]:
                print(f"Кнопка {i+1} нажата!")
                pressed_button_index = i
            
            self.button_states[i] = is_pressed
            
            if is_pressed:
                any_button_pressed = True
        
        # Управляем баннером
        if any_button_pressed:
            # Если нажата новая кнопка, меняем материал
            if pressed_button_index != -1 and pressed_button_index != self.current_button:
                self.set_banner_material(pressed_button_index)
                self.current_button = pressed_button_index
            
            if not self.banner_visible:
                self.set_banner_visibility(True)
                # print(f"Баннер появляется с картинкой для кнопки {self.current_button + 1}!")
        else:
            if self.banner_visible:
                self.set_banner_visibility(False)
                self.current_button = -1
                # print("Баннер скрывается (все кнопки отпущены)!")
    
    def set_banner_visibility(self, visible):
        """Устанавливает видимость баннера"""
        if self.banner_geom_id != -1:
            if visible:
                # Используем текущий материал (уже установлен)
                pass  # Материал уже установлен, просто делаем видимым
            else:
                # Возвращаем материал по умолчанию (скрытый)
                if self.default_material_id != -1:
                    self.model.geom_matid[self.banner_geom_id] = self.default_material_id
            self.banner_visible = visible
    
    def set_banner_material(self, button_index):
        """Устанавливает материал баннера в зависимости от нажатой кнопки"""
        if (self.banner_geom_id != -1 and 
            0 <= button_index < len(self.banner_material_ids) and 
            self.banner_material_ids[button_index] != -1):
            
            self.model.geom_matid[self.banner_geom_id] = self.banner_material_ids[button_index]
            # print(f"Установлен материал для кнопки {button_insdex + 1}")
# Создаем контроллер кнопок и баннера
button_controller = ButtonBannerController(mj_model, mj_data)

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)

def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        # Обновляем состояние кнопок и баннера
        button_controller.update()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)

if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()