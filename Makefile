# Put it first so that "make" without argument is like "make help".
ROOTDIR = $(CURDIR)
version_type := local
EXTRA_ARGS =
PIP_ARGS =
BUILD_ARGS =
COMMIT_UNIXTIME := $(shell git log -n 1 --pretty='format:%ct')
COMMIT_DATETIME := $(shell date -d @${COMMIT_UNIXTIME} +'%Y%m%d%H%M%S')
COMMIT_ID := $(shell git rev-parse --short HEAD)


# Get all args to pass to colcon
# --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# --packages-select my_package to select a specific package

build_args := --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install

# build_args := --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install

extra_args :=

# See Colcon tutorial for more information:
#	https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#
ros2_build:
	@echo "Building ROS2 package..."
	@colcon build $(build_args) $(extra_args)

# See ROS2 Test tutorial for more information:
#	https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html
ros2_test:
	@bash -c "source install/local_setup.bash && \
	export ORCHARD_CI=1 && \
	colcon test --merge-install --return-code-on-test-failure --abort-on-error \
	--event-handlers console_cohesion+ --python-testing pytest \
	--pytest-with-coverage --pytest-args --cov-report=term $(extra_args)"

test-result:
	@bash -c "source install/local_setup.bash && \
	colcon test-result --all --verbose $(extra_args)"

ros2_clean:
	@rm -rf build install log

dev-env:
	@pip3 install -r scm/requirements.txt ${PIP_ARGS}
	@pre-commit install

auto-format:
	@python3 scm/lint/check_lint.py --auto_format

check-lint:
	@python3 scm/lint/check_lint.py
	@pre-commit run check-merge-conflict
	@pre-commit run check-license-header --all-files

show-args:
	@echo "PIP_ARGS: $(PIP_ARGS)"
	@echo "BUILD_ARGS: $(BUILD_ARGS)"
	@echo "EXTRA_ARGS: $(EXTRA_ARGS)"
