"""Portable result serialization.

Pickle is used for Python objects because it preserves nested edge records and
call-free NumPy arrays without silently dropping MATLAB cell/struct contents.  A
JSON metadata sidecar is optional and contains the dimensions/units needed to
inspect a result without loading binary arrays.
"""
from __future__ import annotations
from pathlib import Path
import json
import pickle
import numpy as np
from .model import model_metadata
from .kprm import Roadmap, save_trajectory_graph, load_trajectory_graph


def inspect_legacy_mat_file(path):
    """Inspect a MATLAB cache without making it executable.

    v7 files are inspected with ``scipy.io.whosmat``; v7.3 files are inspected
    with h5py when available.  The function intentionally returns metadata only
    so an eight-thruster cache cannot enter a six-thruster run by accident.
    """
    path=Path(path)
    try:
        from scipy.io import whosmat
        return {"format":"mat-v7","variables":[{"name":n,"shape":tuple(s),"class":c} for n,s,c in whosmat(path)]}
    except Exception as exc:
        try:
            import h5py
            with h5py.File(path,"r") as h:
                return {"format":"mat-v7.3","variables":sorted(h.keys())}
        except Exception:
            return {"format":"unknown","error":str(exc)}


def save_result(path, result):
    path=Path(path)
    payload={"format":"bluerov2-python-result-v1","result":result}
    with path.open("wb") as f:pickle.dump(payload,f,protocol=pickle.HIGHEST_PROTOCOL)
    meta={"format":payload["format"],"success":bool(result.get("success",False)),"model":model_metadata(result["model"]),
          "state_shape":list(np.asarray(result.get("nominalTrajectory",np.empty((12,0)))).shape),
          "control_shape":list(np.asarray(result.get("controlTrajectory",np.empty((6,0)))).shape),
          "time_length":len(result.get("time",[]))}
    path.with_suffix(path.suffix+".json").write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding="utf-8")


def load_result(path):
    with Path(path).open("rb") as f:payload=pickle.load(f)
    result=payload.get("result",payload)
    if result.get("model").num_thrusters != 6:raise ValueError("eight-thruster result cannot be loaded by six-thruster executor")
    return result


def save_json_metadata(path, result):
    path=Path(path);path.write_text(json.dumps({"model":model_metadata(result["model"]),"success":bool(result.get("success",False))},ensure_ascii=False,indent=2),encoding="utf-8")

