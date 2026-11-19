"""Ensure discussion references do not silently disappear before paper writing."""
import json
from pathlib import Path


def test_catalog_has_primary_metadata_and_matching_bibtex():
    root = Path(__file__).resolve().parents[1] / 'references'
    entries = json.loads((root/'catalog.json').read_text())['entries']
    bib = (root/'references.bib').read_text()
    keys = [e['key'] for e in entries]
    assert len(keys) == len(set(keys))
    assert {'lu2026aspire','ace2026','schlesinger2024roborepair','fu2026capx','ahn2025reliablecap'} <= set(keys)
    for e in entries:
        assert e['authors'] and e['metadata_status']=='verified_primary_metadata'
        assert e['url'].startswith('https://arxiv.org/abs/')
        assert '@misc{'+e['key']+',' in bib
        assert e['arxiv_id'] in bib
        assert e['reading_status'] and 'reproduced' in e
