Files written by the rby1 web backend's `config_backup.py` (rby1 `origin/dev` 8571cbf) with the fake robot of
its `test_config_backup.py`, so `test_config_backup.cpp` checks that this package reads web files and previews
them as the web does.

- `web_export.json`: `build_export` (exported_at and hostname fixed).
- `web_modified.json`: the same file with a changed scaling, hotspot SSID, robot_model_name, boot target, an
  unknown parameter and a type mismatch.
- `web_preview_parameters.json`: `build_preview(...)["parameters"]` of `web_modified.json` on another robot.
