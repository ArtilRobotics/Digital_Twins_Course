import os
from setuptools import find_packages, setup

package_name = 'webots_robot_control'

# Lista de carpetas que quieras copiar
folders_to_include = ['mini4DoF', 'launch', 'resource']  # Añade las carpetas que quieras incluir aquí

data_files = [
    ('share/' + package_name, ['package.xml'])  # Esto copia el package.xml
]

# Procesar cada carpeta listada y copiar todo su contenido (archivos y subcarpetas)
for folder in folders_to_include:
    for root, dirs, files in os.walk(folder):
        # Mantener la estructura relativa en 'share/package_name/<carpeta>'
        install_dir = os.path.join('share', package_name, root)
        
        # Obtener la lista de archivos en la carpeta actual
        file_paths = [os.path.join(root, file) for file in files]
        
        # Solo agregar si hay archivos en la carpeta actual
        if file_paths:
            data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='digital',
    maintainer_email='digital@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'webots_control_node = webots_robot_control.webots_control_node:main',
        ],
    }
)
