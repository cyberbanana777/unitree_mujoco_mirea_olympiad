# Обзор

Данный репозиторий представляет собой доработку [Unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco.git). Решение включет в себя изменение сцены симуляции и внедрение испытательного стенд с кнопками для реализации кейса олимпиады по робототехнике.


# Отличия от оригинального репозитория

- Добавлена новая сцена (загрузается по умолчанию)
- Добавлена новая механика для взаимодействия с кнопками на испытательнном стенде.


# Введение
## Unitree mujoco
"unitree_mujoco" - это симулятор, разработанный на основе `Unitree sdk2` и `mujoco`. Пользователи могут легко интегрировать управляющие программы, разработанные с помощью `Unitree_sdk2`, `unitree_ros2` и `unitree_sdk2_python`, в этот симулятор, обеспечивая плавный переход от моделирования к физическому развитию. Репозиторий содержит две версии симулятора, реализованные на C++ и Python, со следующей структурой:
![](./doc/func.png)

## Структура каталогов
- `simulate`: симулятор реализован на основе `unitree_sdk2` и `mujoco` (рекомендуется C++).
- `simulate_python`: симулятор реализован на основе `unitree_sdk2_python` и `mujoco` (Python).
- `unitree_robots`: файлы описания MJCF для роботов, поддерживаемых `unitree_sdk2`
- `terrain_tool`: Инструмент для создания ландшафта в сценариях моделирования
- `example`: Примеры программ

## Поддерживаются сообщения Unitree sdk2:
**Текущая версия поддерживает только низкоуровневую разработку, в основном используется для симуляции реальной проверки контроллера**
- `LowCmd`: команды управления двигателем
- `LowState`: информация о состоянии двигателя
- `SportModeState`: данные о местоположении и скорости робота

Примечание:
1. Нумерация двигателей соответствует фактическому оборудованию робота. Более подробную информацию можно найти в [документации Unitree](https://support.unitree.com/home/zh/developer).
2. В реальном оборудовании робота сообщение `SportModeState` не читается после отключения встроенной службы управления движением. Однако тренажер сохраняет это сообщение, чтобы пользователи могли использовать информацию о местоположении и скорости для анализа разработанных программ управления.

## Related links
- [Оригинальный Unitree_mujoco, который лежит в основе данного репозитория](https://github.com/unitreerobotics/unitree_mujoco.git)
- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- [Unitree Doc](https://support.unitree.com/home/zh/developer)
- [Mujoco Doc](https://mujoco.readthedocs.io/en/stable/overview.html)

## Описание типа сообщения (idl DDS)
- Роботы Unitree Go2, B2, H1, B2w, Go2w используют idl unitree_go для низкоуровневой связи.
- Робот Unitree G1, H1-2 использует unitree_hg idl для низкоуровневой связи.

# Установка

## Python симулятор (simulate_python)

### 1. Зависимости

#### unitree_sdk2_python
```bash
cd ~
sudo apt install python3-pip
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
Если вы столкнетесь с ошибкой во время установки:
```bash
Could not locate cyclonedds. Try to set CYCLONEDDS_HOME or CMAKE_PREFIX_PATH
```
решение ищите здесь: https://github.com/unitreerobotics/unitree_sdk2_python

#### mujoco-python
```bash
pip3 install mujoco
```

#### joystick
```bash
pip3 install pygame
```

### 2. Тестирвоание и запуск
```bash
git clone https://github.com/cyberbanana777/unitree_mujoco_mirea_olympiad.git
cd ./unitree_mujoco_mirea_olympiad/simulate_python
python3 ./unitree_mujoco.py
```

Если установка прошла успешно, то Вы увидете следующиее (для просмотра перйдите по ссылке):

[Корректная работа робота в симуляции](https://disk.yandex.ru/i/a1jsti4PgwaFvA)

Более подробную инструкцию по использованию симмулятора и программирвоанию роботов можно найти в [оригинальной документации](https://github.com/unitreerobotics/unitree_mujoco.git).