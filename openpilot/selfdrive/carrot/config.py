#!/usr/bin/env python3
"""
统一的参数管理模块 (从 CarrotPilot 移植)
提供统一的接口访问系统参数和自定义参数，支持fallback机制
"""

import json
import os
from openpilot.common.params import Params

try:
  from openpilot.common.params_pyx import UnknownKeyName
except ImportError:
  UnknownKeyName = KeyError


class UnifiedParams:
    """统一的参数管理类，同时处理系统参数和自定义参数"""

    _instance = None
    _initialized = False

    def __new__(cls, nav_json_file=None):
        if cls._instance is None:
            cls._instance = super(UnifiedParams, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, nav_json_file=None):
        if self._initialized:
            return

        self.system_params = Params()

        if nav_json_file is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            nav_json_file = os.path.join(current_dir, "nav_params.json")

        self.nav_json_file = os.path.realpath(nav_json_file)
        self.nav_data = {}
        self._load_nav_params()
        self._initialized = True

    def _match_system_param(self):
      for key in list(self.nav_data.keys()):
        try:
          sys_val = None
          if isinstance(self.nav_data[key], int):
            sys_val = self.system_params.get_int(key)
          elif isinstance(self.nav_data[key], float):
            sys_val = self.system_params.get_float(key)
          elif self.nav_data[key] in (0, 1):
            sys_val = self.system_params.get_bool(key)

          if sys_val is not None:
            self.nav_data[key] = sys_val
        except Exception:
          pass

    def _save_system_param(self):
      for key, value in list(self.nav_data.items()):
        try:
          if isinstance(value, bool) or value in (0, 1):
            self.system_params.put_bool(key, bool(value))
          elif isinstance(value, int):
            self.system_params.put_int(key, int(value))
          elif isinstance(value, float):
            self.system_params.put_float(key, float(value))
          else:
            self.system_params.put(key, str(value))
        except (KeyError, AttributeError, UnknownKeyName):
          pass
        except Exception:
          pass

    def _load_nav_params(self):
        try:
            if os.path.exists(self.nav_json_file):
                with open(self.nav_json_file, 'r', encoding='utf-8') as f:
                    self.nav_data = json.load(f)
                    self._match_system_param()
            else:
                self.nav_data = self._get_default_nav_data()
                self._match_system_param()
                self._save_nav_data()
        except json.JSONDecodeError as e:
            print(f"Config file format error, using defaults: {e}")
            self.nav_data = self._get_default_nav_data()
            self._match_system_param()
            self._save_nav_data()
        except (OSError, IOError) as e:
            print(f"Failed to load params {self.nav_json_file}: {e}")
            self.nav_data = self._get_default_nav_data()
            self._match_system_param()

    def _save_nav_data(self):
        try:
            os.makedirs(os.path.dirname(self.nav_json_file), exist_ok=True)
            with open(self.nav_json_file, 'w', encoding='utf-8') as f:
                json.dump(self.nav_data, f, indent=2, ensure_ascii=False)
        except (OSError, IOError) as e:
            print(f"Failed to save params: {e}")

    def _get_default_nav_data(self):
        return {
            "AutoTurnDistOffset": 0,
            "AutoForkDistOffset": 30,
            "AutoDoForkBlinkerDist": 15,
            "AutoDoForkNavDist": 15,
            "AutoForkDistOffsetH": 1000,
            "AutoDoForkDecalDistH": 50,
            "AutoDoForkDecalDist": 20,
            "AutoDoForkBlinkerDistH": 30,
            "AutoDoForkNavDistH": 50,
            "AutoUpRoadLimit": 0,
            "AutoUpRoadLimit40KMH": 15,
            "AutoUpHighwayRoadLimit": 0,
            "AutoUpHighwayRoadLimit40KMH": 15,
            "RoadType": -1,
            "AutoForkDecalRateH": 80,
            "AutoForkSpeedMinH": 60,
            "AutoKeepForkSpeedH": 5,
            "AutoForkDecalRate": 80,
            "AutoForkSpeedMin": 45,
            "AutoKeepForkSpeed": 5,
            "ShowDebugLog": 0,
            "AutoCurveSpeedFactorH": 100,
            "AutoCurveSpeedAggressivenessH": 100,
            "SameSpiCamFilter": 1,
            "StockBlinkerCtrl": 0,
            "ExtBlinkerCtrlTest": 0,
            "BlinkerMode": 1,
            "LaneStabTime": 50,
            "DynamicBlindRange": 0,
            "DynamicBlindDistance": 0,
            "DisableBlindSpot": 0,
            "BsdDelayTime": 20,
            "SideBsdDelayTime": 20,
            "SideRelDistTime": 10,
            "SidevRelDistTime": 10,
            "SideRadarMinDist": 0,
            "AutoTurnInNotRoadEdge": 1,
            "ContinuousLaneChange": 1,
            "ContinuousLaneChangeCnt": 4,
            "ContinuousLaneChangeInterval": 2,
            "AutoTurnLeft": 1,
            "AutoEnTurnNewLaneTimeH": 0,
            "AutoEnTurnNewLaneTime": 0,
            "NewLaneWidthDiff": 8,
        }

    def get_bool(self, key, default=False):
        try:
            value = self.system_params.get_bool(key)
            if value is not None:
                return bool(value)
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return bool(int(value))
            except (ValueError, TypeError):
                return default
        return default

    def get_int(self, key, default=0):
        try:
            value = self.system_params.get(key)
            if value is not None:
                return int(value)
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return int(value)
            except (ValueError, TypeError):
                return default
        return default

    def get_float(self, key, default=0.0):
        try:
            value = self.system_params.get(key)
            if value is not None:
                return float(value)
        except (KeyError, AttributeError, UnknownKeyName):
            pass

        if key in self.nav_data:
            value = self.nav_data[key]
            try:
                return float(value)
            except (ValueError, TypeError):
                return default
        return default

    def put_bool(self, key, value):
      bool_value = bool(value)
      int_value = int(bool_value)
      json_need_save = False

      try:
        self.system_params.put_bool(key, bool_value)
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = int_value
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = int_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put_int(self, key, value):
      int_value = int(value)
      json_need_save = False

      try:
        self.system_params.put(key, str(int_value))
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = int_value
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = int_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put_float(self, key, value):
      float_value = float(value)
      json_need_save = False

      try:
        self.system_params.put(key, str(float_value))
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = float_value
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = float_value
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def put(self, key, dat):
      json_need_save = False

      try:
        self.system_params.put(key, dat)
      except (KeyError, AttributeError, UnknownKeyName):
        self.nav_data[key] = dat
        json_need_save = True
      else:
        if key in self.nav_data:
          self.nav_data[key] = dat
          json_need_save = True

      if json_need_save:
        try:
          self._save_nav_data()
        except (OSError, IOError):
          pass

    def get(self, key, encoding=None):
        val = self.system_params.get(key)
        if val is not None:
            if encoding and isinstance(val, bytes):
                return val.decode(encoding)
            return val
        return None

    def remove(self, key):
        try:
            self.system_params.remove(key)
        except Exception:
            pass


# Global singleton instance
unified_params = UnifiedParams()
