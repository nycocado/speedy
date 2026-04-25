cmake_minimum_required(VERSION 3.8)
project(speedy_teleop)

# Ativa avisos do compilador para garantir a qualidade do código
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# Dependências
# -----------------------------------------------------------------------------

# Busca os pacotes necessários do ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# -----------------------------------------------------------------------------
# Instalação
# -----------------------------------------------------------------------------

# Exporta as pastas de configuração e launch para a pasta 'share' do workspace.
# Isso permite que comandos como 'ros2 launch' e 'ros2 run' localizem os recursos.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testes (Opcional)
# -----------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Ignora verificações de copyright durante o desenvolvimento inicial
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza o pacote ament (obrigatório para integração com o workspace)
ament_package()
