Release checklist
-------------------

- catkin_generate_changelog
- catkin_prepare_release
- bloom-release

The version is only stored in `package.xml`: `mvsim_version.h` is generated
into the build tree at configure time, so releases never need to commit it.
bloom tracks use `develop` as the upstream branch (there is no `master`).
