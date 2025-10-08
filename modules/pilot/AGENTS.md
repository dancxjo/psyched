# Pilot module agent notes

- When running Python unit tests from this module, set `PYTHONPATH=modules/pilot/packages/pilot` so that the `pilot` package and
  its `sitecustomize.py` shim are importable without installing the wheel. Run the narrowest set of tests possible—ROS tooling
  such as `ament_*` linters are optional and can stay out of the loop for targeted unit coverage.
