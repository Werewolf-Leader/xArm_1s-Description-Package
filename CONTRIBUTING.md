# Contributing to XArm ROS2 URDF Package

Thank you for your interest in contributing to this project! We welcome contributions from the community.

## How to Contribute

### Reporting Issues

If you find a bug or have a suggestion for improvement:

1. Check if the issue already exists in the [Issues](../../issues) section
2. If not, create a new issue with:
   - Clear title and description
   - Steps to reproduce (for bugs)
   - Expected vs actual behavior
   - ROS2 distribution and OS version
   - Any relevant error messages or logs

### Submitting Changes

1. **Fork the Repository**
   - Click the "Fork" button at the top right of the repository page

2. **Clone Your Fork**
   ```bash
   git clone https://github.com/YOUR_USERNAME/xArm_1s-ROS2-URDF-package.git
   cd xArm_1s-ROS2-URDF-package
   ```

3. **Create a Branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

4. **Make Your Changes**
   - Follow ROS2 coding standards
   - Test your changes thoroughly
   - Update documentation if needed

5. **Build and Test**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   # Test your changes
   ros2 launch xarm_description display.launch.py
   ```

6. **Commit Your Changes**
   ```bash
   git add .
   git commit -m "Add: Brief description of your changes"
   ```
   
   Commit message format:
   - `Add:` for new features
   - `Fix:` for bug fixes
   - `Update:` for updates to existing features
   - `Docs:` for documentation changes
   - `Refactor:` for code refactoring

7. **Push to Your Fork**
   ```bash
   git push origin feature/your-feature-name
   ```

8. **Create a Pull Request**
   - Go to the original repository
   - Click "New Pull Request"
   - Select your fork and branch
   - Provide a clear description of your changes

## Development Guidelines

### Code Style

- Follow [ROS2 style guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use meaningful variable and function names
- Comment complex logic
- Keep functions focused and modular

### URDF/XACRO Files

- Maintain proper XML formatting
- Use xacro macros for repeated elements
- Include comments for complex transformations
- Validate URDF: `check_urdf <urdf_file>`

### Launch Files

- Use Python launch files (not XML)
- Include argument descriptions
- Use proper event handlers for sequencing
- Test launch files before submitting

### Testing

Before submitting a PR, ensure:
- Package builds without errors: `colcon build`
- No syntax errors in Python/URDF files
- Launch files work correctly
- Robot displays properly in RViz2
- Gazebo simulation runs (if modified)

## Areas for Contribution

We especially welcome contributions in these areas:

- **MoveIt2 Integration**: Configuration for motion planning
- **Hardware Interface**: Real robot communication
- **Documentation**: Tutorials, examples, troubleshooting
- **Testing**: Unit tests, integration tests
- **Bug Fixes**: Any issues you encounter
- **Performance**: Optimization improvements

## Questions?

If you have questions about contributing:
- Open a [Discussion](../../discussions)
- Check existing documentation
- Review closed issues and PRs

## Code of Conduct

- Be respectful and inclusive
- Provide constructive feedback
- Focus on the code, not the person
- Help others learn and grow

## License

By contributing, you agree that your contributions will be licensed under the same Creative Commons Attribution-NonCommercial 4.0 license that covers this project.

---

Thank you for helping improve this project! 🤖
