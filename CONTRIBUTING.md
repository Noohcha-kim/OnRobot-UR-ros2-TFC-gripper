# Contributing Guide

Thank you for contributing to OnRobot Gripper ROS2 Driver! 🎉

---

## 📋 How to Contribute

### 1. Bug Reports

Found a bug? Please report it on [GitHub Issues](https://github.com/Noohcha-kim/onrobot_gripper_driver/issues).

#### Bug Report Template

```markdown
### Environment
- ROS2 Version: Humble
- OS: Ubuntu 22.04
- UR Robot Model: UR5e
- Gripper Model: 2FG7
- Driver Version: v1.0.0

### Problem Description
Clearly and concisely describe the issue.

### Steps to Reproduce
1. Do this
2. Do that
3. Error occurs

### Expected Behavior
What should happen?

### Actual Behavior
What actually happened?

### Logs
```
Error logs here
```

### Additional Information
Screenshots, videos, etc.
```

---

### 2. Feature Requests

Want to suggest a new feature?

1. Check if similar suggestions exist on [GitHub Issues](https://github.com/Noohcha-kim/onrobot_gripper_driver/issues)
2. If not, create a new Issue
3. Include **[Feature Request]** tag with:
   - Why is this feature needed?
   - How should it work?
   - Usage examples

---

### 3. Pull Requests

Want to contribute code?

#### Process

1. **Fork** this repository

2. **Clone** your fork
```bash
git clone https://github.com/Noohcha-kim/onrobot_gripper_driver.git
cd onrobot_gripper_driver
```

3. **Create Branch**
```bash
git checkout -b feature/your-feature-name
# or
git checkout -b bugfix/issue-number-description
```

4. **Write Code**
   - C++ code: Follow [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
   - Python code: Follow PEP8
   - Comments should be clear and descriptive

5. **Test**
```bash
cd ~/ros2_ws
colcon build --packages-select onrobot_gripper_driver
source install/setup.bash

# Test with actual gripper
ros2 launch onrobot_gripper_driver onrobot_gripper.launch.py
```

6. **Commit**
```bash
git add .
git commit -m "feat: Add support for XYZ gripper"
```

**Commit Message Conventions:**
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `style:` Code formatting
- `refactor:` Code refactoring
- `test:` Add tests
- `chore:` Build/configuration changes

7. **Push**
```bash
git push origin feature/your-feature-name
```

8. **Create Pull Request**
   - Create PR on GitHub
   - Clear title and description
   - Link related Issues (if any)

---

## 🧪 Testing Guide

### Hardware Testing

Always test new features or bug fixes with actual gripper:

1. **Basic Operation Test**
```bash
# Grip
ros2 service call /onrobot_gripper_node/grip onrobot_gripper_driver/srv/Grip \
  "{target_position: 50.0, force: 40.0, speed: 50.0}"

# Release
ros2 service call /onrobot_gripper_node/release onrobot_gripper_driver/srv/Release "{}"
```

2. **Range Test**
```bash
# Minimum values
ros2 service call /onrobot_gripper_node/grip onrobot_gripper_driver/srv/Grip \
  "{target_position: 0.0, force: 20.0, speed: 10.0}"

# Maximum values
ros2 service call /onrobot_gripper_node/grip onrobot_gripper_driver/srv/Grip \
  "{target_position: 110.0, force: 120.0, speed: 100.0}"
```

3. **Error Handling Test**
```bash
# Kill socat (simulate connection loss)
killall socat

# Send command (check retry logic)
ros2 service call /onrobot_gripper_node/grip ...

# Restart socat
./start_socat.sh
```

4. **Long-term Stability Test**
```bash
# 100 cycles
for i in {1..100}; do
  echo "[$i/100]"
  ros2 service call /onrobot_gripper_node/grip ... > /dev/null
  sleep 2
  ros2 service call /onrobot_gripper_node/release ... > /dev/null
  sleep 2
done
```

---

## 📝 Coding Style

### C++

```cpp
// Class names: PascalCase
class GripperDriver {
public:
    // Function names: snake_case
    bool initialize();
    
    // Variable names: snake_case
    double target_position_;
    
    // Constants: UPPER_CASE
    static constexpr int MAX_RETRIES = 3;
    
private:
    // Private variables with _ suffix
    std::string device_name_;
};
```

### Python

```python
# Class names: PascalCase
class GripperController:
    def __init__(self):
        # Variable names: snake_case
        self.target_position = 0.0
        
        # Private variables with _ prefix
        self._client = None
    
    # Function names: snake_case
    def grip_object(self, width, force):
        pass
```

---

## 🔍 Code Review

Pull Requests will be reviewed based on:

1. **Functionality**
   - Works as intended?
   - Edge cases handled?

2. **Code Quality**
   - Easy to read and understand?
   - Appropriate comments?
   - No code duplication?

3. **Testing**
   - Tested with actual hardware?
   - Works in various situations?

4. **Documentation**
   - README updates needed?
   - New features documented?

5. **Compatibility**
   - Doesn't break existing code?
   - Works on ROS2 Humble?

---

## 🚫 Don'ts

- **Hardcoded values**: Use parameters for IPs, paths, etc.
- **Huge PRs**: Break into smaller units
- **Code without tests**: Always test with actual hardware
- **Complex code without comments**: Explain intent clearly
- **Breaking Changes**: If necessary, clearly indicate

---

## 💬 Questions?

- Post questions on GitHub Issues
- Use Discussion tab
- Email: minseung2201@gmail.com

---

## 🎖️ Contributors

All contributors will be listed in README!

---

**Thank you again!** Your contributions make this project better! 🚀
