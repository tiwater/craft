"""Pure-stdlib HTTP client for the standalone EvidenceScene v3 Spatial State Runtime Service.

Consumes the transport-independent EvidenceScene v3 Machine Spatial API (`/v1/*`)
without importing any EvidenceScene internal Python/NumPy/C11 modules into TRAX core.
"""
from __future__ import annotations

import json
from typing import Any, Mapping, Sequence
from urllib.parse import urlencode
import urllib.request


class EvidenceSceneClient:
    """Thin pure-stdlib HTTP client for EvidenceScene v3 (`/v1/*`)."""

    def __init__(self, base_url: str, *, timeout_s: float = 12.0) -> None:
        cleaned = (base_url or "").strip().rstrip("/")
        if not cleaned:
            raise ValueError("EvidenceScene base_url must be non-empty")
        self.base_url = cleaned
        self.timeout_s = float(timeout_s)
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _get_json(self, path: str, query: Mapping[str, Any] | None = None) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        if query:
            qs = urlencode({k: v for k, v in query.items() if v is not None})
            if qs:
                url = f"{url}?{qs}"
        with self._opener.open(url, timeout=self.timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def _get_bytes(self, path: str) -> tuple[bytes, str, str]:
        url = f"{self.base_url}{path}"
        with self._opener.open(url, timeout=self.timeout_s) as resp:
            data = resp.read()
            mime = resp.headers.get("Content-Type") or "application/octet-stream"
            etag = resp.headers.get("ETag") or ""
            return data, mime, etag

    def _post_json(self, path: str, body: Mapping[str, Any] | None = None) -> dict[str, Any]:
        url = f"{self.base_url}{path}"
        raw = json.dumps(dict(body or {})).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=raw,
            method="POST",
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
        with self._opener.open(req, timeout=self.timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def health(self) -> dict[str, Any]:
        return self._get_json("/v1/health")

    def get_snapshot(self, *, refresh: bool = False) -> dict[str, Any]:
        return self._get_json("/v1/snapshot", {"refresh": "1" if refresh else "0"})

    def get_surfels(self) -> dict[str, Any]:
        return self._get_json("/v1/surfels")

    def get_surface_estm(self, owner_id: str = "tabletop") -> tuple[bytes, str, str]:
        return self._get_bytes(f"/v1/surfaces/{owner_id}.estm")

    def changes_since(self, *, scene_id: str, epoch: int, revision: int) -> dict[str, Any]:
        return self._get_json("/v1/changes", {"scene_id": scene_id, "epoch": int(epoch), "revision": int(revision)})

    def sync(self) -> dict[str, Any]:
        return self._post_json("/v1/sync", {})

    def reset(self, *, reason: str = "host_reset") -> dict[str, Any]:
        return self._post_json("/v1/reset", {"reason": reason})

    def ingest_rgbd(self, bundle: Mapping[str, Any]) -> dict[str, Any]:
        return self._post_json("/v1/ingest/rgbd", bundle)

    def assess_spatial_requirements(
        self,
        *,
        entity_id: str = "red_cylinder",
        task_mode: str = "body_grasp",
        reference_clock_id: str | None = None,
        reference_ns: int | None = None,
        max_age_ms: float = 2000.0,
        max_stddev_m: float | None = None,
        entity_point_m: Sequence[float] | None = None,
        refresh: bool = True,
    ) -> dict[str, Any]:
        return self._post_json(
            "/v1/assess",
            {
                "entity_id": entity_id,
                "task_mode": task_mode,
                "reference_clock_id": reference_clock_id,
                "reference_ns": reference_ns,
                "max_age_ms": float(max_age_ms),
                "max_stddev_m": max_stddev_m,
                "entity_point_m": list(entity_point_m) if entity_point_m is not None else None,
                "refresh": bool(refresh),
            },
        )

    def revalidate_spatial_requirements(
        self,
        assessment_id: str,
        *,
        reference_clock_id: str | None = None,
        reference_ns: int | None = None,
    ) -> dict[str, Any]:
        return self._post_json(
            "/v1/revalidate",
            {
                "assessment_id": str(assessment_id),
                "reference_clock_id": reference_clock_id,
                "reference_ns": reference_ns,
            },
        )

    def evaluate_viewpoints(
        self,
        assessment_id: str,
        *,
        candidates_xyz_m: Sequence[Sequence[float]] | None = None,
        reference_xyz_m: Sequence[float] | None = None,
    ) -> dict[str, Any]:
        return self._post_json(
            "/v1/evaluate_viewpoints",
            {
                "assessment_id": str(assessment_id),
                "candidates_xyz_m": [list(c) for c in candidates_xyz_m] if candidates_xyz_m is not None else None,
                "reference_xyz_m": list(reference_xyz_m) if reference_xyz_m is not None else None,
            },
        )

    def select_surface_target(
        self,
        *,
        entity_id: str = "auto",
        point_index: int | None = None,
        target_xyz_m: Sequence[float] | None = None,
        reviewer_id: str = "operator",
        reason: str = "trax_surface_selection",
    ) -> dict[str, Any]:
        return self._post_json(
            "/v1/select_surface",
            {
                "entity_id": str(entity_id),
                "point_index": point_index,
                "target_xyz_m": list(target_xyz_m) if target_xyz_m is not None else None,
                "reviewer_id": str(reviewer_id),
                "reason": str(reason),
            },
        )
