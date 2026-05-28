# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from conan import ConanFile
from conan.tools.cmake import CMake, CMakeDeps, CMakeToolchain, cmake_layout
from conan.tools.env import VirtualBuildEnv

required_conan_version = ">=2.2.2"


class app(ConanFile):
    settings = "compiler", "build_type", "os", "arch"
    options = {
        "platform": ["ANY"],
    }
    default_options = {
        "platform": "unspecified",
    }

    def set_version(self):
        # Use latest if not specified via command line
        if not self.version:
            self.version = "latest"

    def requirements(self):
        self.requires("libhal-util/[^5.4.0]")
        ARCHITECTURE = str(self.settings.arch)
        if ARCHITECTURE.startswith("cortex-m"):
            self.output.warning(
                "libhal-arm-mcu usable platform detected...")
            self.requires("libhal-arm-mcu/[^1.0.0]")
        else:
            self.output.warning("No platform library added...")

    def layout(self):
        BUILD_PATH = Path("build") / str(self.options.platform)
        cmake_layout(self, build_folder=BUILD_PATH)

    def build_requirements(self):
        self.tool_requires("ninja/[^1.0.0]")
        self.tool_requires("cmake/[^4.0.0]")
        self.tool_requires("libhal-cmake-util/[^4.3.3]")

    def generate(self):
        virt = VirtualBuildEnv(self)
        virt.generate()

        cmake = CMakeDeps(self)
        cmake.generate()

        tc = CMakeToolchain(self)
        tc.generator = "Ninja"
        tc.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.configure(variables={"E10_ADAPTER_VERSION": str(self.version)})
        cmake.build()