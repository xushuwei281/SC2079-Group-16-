# MDP dataset audit

Source: `${CV_WORKSPACE}/data-smoke/20260908T142129Z`

Result: **STRUCTURAL CHECKS PASS**

not supplied; augmentation/session separation UNVERIFIED

| Split | Images | Valid labels | Empty labels | Objects |
|---|---:|---:|---:|---:|
| train | 6765 | 6765 | 0 | 6765 |
| valid | 757 | 757 | 0 | 757 |
| test | 324 | 324 | 0 | 324 |

## file_sha256 duplicates

Groups: 0; cross-split groups: 0; extra copies: 0.

## decoded_rgb_sha256 duplicates

Groups: 0; cross-split groups: 0; extra copies: 0.

## Errors

None.

## Warnings

None.

## Limits and manual review

- Exact-file and decoded-pixel hashes cannot identify general augmentations, near duplicates, or adjacent video frames.
- Without complete, independently verified original/session groups, split independence remains unverified.
- A groups CSV enforces only the provenance supplied by its author; this tool cannot verify those group assignments.
- Empty labels are accepted as background negatives; inspect their images to confirm there are no unlabeled targets.
- Polygon checks cover syntax, finite normalized coordinates, nonzero bounds and area; they do not prove annotation semantics or absence of self-intersection.
- Decode and hash auditing does not establish that classes, target direction, or annotations are visually correct.
- Freeze and hash the resulting snapshot before training. Audit a stable local copy, not a source being changed concurrently.

Per-image hashes, annotation digests, duplicate membership, and per-class counts are in the JSON manifest.
