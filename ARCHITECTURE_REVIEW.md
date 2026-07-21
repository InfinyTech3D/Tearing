# Tearing — Architecture / Bug / Optimization Review

Initial pass over the plugin (~4.5k lines across `src/Tearing`, its `Controllers/` subfolder, and
`Tearing_test`), done by reading the source directly plus one real incremental build to capture
actual compiler warnings. Nothing has been fixed yet — this is the task list to work from, ranked
roughly by severity within each section. Each item cites file:line as of this pass; re-check before
acting, since line numbers drift.

## 1. Bugs

### 1.1 `TearingEngine::algoFracturePath()` cuts using uninitialized/never-computed endpoints — CRITICAL
`TearingEngine.inl:64-94`:
```cpp
void TearingEngine<DataTypes>::algoFracturePath()
{
    ...
    Coord Pb;
    Coord Pc;
    this->m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, m_maxStressTriangleIndex, principalStressDirection, d_input_positions.getValue());
    ...
}
```
`Pb`/`Pc` are declared with no initializer and never assigned before being passed **by value** into
`TearingAlgorithms::algoFracturePath()` (`Coord Pb, Coord Pc` — by-value params, `TearingAlgorithms.h:93`).
That function uses them throughout to build the actual cut direction (`backward = Pb - Pa`,
`forward = Pc - Pa`, `TearingAlgorithms.inl:134-135`) — so every fracture this engine ever performs
uses a default-constructed (likely zero) `Pb`/`Pc` instead of the real, meaningfully-computed
fracture endpoints.

The correct endpoints *are* computed correctly, just one call earlier and then thrown away:
`computeFracturePath()` (`TearingEngine.inl:98-133`, called every simulation step from
`BaseTearingEngine::handleEvent()`) computes real `Pb`/`Pc` via `computeEndPointsNeighboringTriangles()`
or `computeEndPoints()`, stores them into `this->m_fracturePath.ptB`/`ptC`
(`type::Vec3`, `TearingEngine.inl:126-128`), then calls
`this->m_tearingAlgo->computeFracturePath(this->m_fracturePath)` — which is a **completely empty
stub**:
```cpp
// TearingAlgorithms.inl:119-123
template <class DataTypes>
void TearingAlgorithms<DataTypes>::computeFracturePath(FracturePath& my_fracturePath)
{
}
```
(confirmed by the real compiler warning `C4100: 'my_fracturePath': unreferenced formal parameter`,
`TearingAlgorithms.inl:120`). So the computed `Pb`/`Pc` never leave `m_fracturePath`, and
`algoFracturePath()` — called separately, every `d_stepModulo` steps, or directly via the `'C'`
keypress bypass (`BaseTearingEngine.inl:638-641`) — has no path to read them; it just declares fresh
empty locals instead.

Contrast with `TearingScenarioEngine::algoFracturePath()` (`TearingScenarioEngine.inl:116-139`),
which does this correctly and even shows the fix in place:
```cpp
//Coord Pb, Pc;
computeEndPoints(m_Pa, dir, m_Pb, m_Pc);   // m_Pa/m_Pb/m_Pc are persistent members
...
tearingAlgo->algoFracturePath(m_Pa, indexA, m_Pb, m_Pc, triID, normal, ...);
```
The commented-out `//Coord Pb, Pc;` line is a strong signal this exact mistake was caught and fixed
here, but the fix never made it into `TearingEngine`.

**This is very likely the root cause of `Tearing_test/TearingEngine_test.cpp:167`**, where the only
test that actually exercises tearing is disabled with:
```cpp
// TODO epernod 2026-07-10: test not working anymore. Disabled for the ci but need to be investigated and fixed
//ASSERT_TRUE(this->testTearing());
```
(dated 11 days before this review). The test expects triangle/edge/point counts to change after 100
steps of simulated tearing (1450→1505 triangles etc.) — exactly the kind of check that would fail if
every cut collapses toward a degenerate zero-length direction.

**Task:** have `TearingEngine::algoFracturePath()` read `this->m_fracturePath.ptB`/`ptC` (converting
back to `Coord`) instead of empty locals, or — more robustly, since `algoFracturePath()` can run on
a different cadence than `computeFracturePath()` (including the keypress bypass, which doesn't call
`computeFracturePath()` first) — have it recompute fresh endpoints itself via
`computeFractureDirection()` + `computeEndPointsNeighboringTriangles()`/`computeEndPoints()`,
mirroring what `computeFracturePath()` already does. Then re-enable and fix
`TearingEngine_test.testTearing`.

### 1.2 `TearingAlgorithms::computeFracturePath(...)` (both overloads) are empty stubs
`TearingAlgorithms.inl:111-123` — both the `FracturePath&` overload (used by `TearingEngine`, see
§1.1) and the older `(Coord, Index, Coord, Coord)` overload are empty function bodies. Confirmed by
compiler warnings for every unreferenced parameter (`TearingAlgorithms.inl:112,120`). The
`FracturePath`/`PointToAdd`-based API added in recent history (git log: "Add new FracturePath
structure using new PointToAdd classes (#67)", "Fix the logic when to start tearing and add
fracture structure (#65)") appears to be an unfinished refactor — the struct and its `pointsToAdd`/
`pathOk` fields exist and get populated with `ptA`/`ptB`/`ptC`/`triIdA`, but nothing ever computes
`pointsToAdd` or flips `pathOk` to `true`. This is the same gap as §1.1, from the algorithm side.

**Task:** either finish implementing this overload (computing the actual point/split list into
`my_fracturePath.pointsToAdd` and setting `pathOk`), or remove it and the dead
`(Coord, Index, Coord, Coord)` overload if the intent is to keep using the older
`algoFracturePath(...)` signature going forward — right now the plugin ships both an unfinished and
a working path side by side with no indication which one is authoritative.

### 1.3 `VolumeTearingEngine`'s tetrahedron-stress detection and cutting-plane computation are commented out
Matches the README's own disclosure ("Volume tearing... still work in progress", TRL 3) — not a
surprise, but worth cataloging precisely for whoever picks this up:
- `VolumeTearingEngine.inl:352-385`: the entire loop body that would populate `candidate`,
  `candidateVonMises`, and `indexTetraMaxStress` from `tetrahedronInfo->maxStress`/`vonMisesStress`
  is commented out (`// TODO: restore that later`, line 356). `indexTetraMaxStress` stays at its
  default `-1` forever; the candidate lists stay empty.
- `computePlane()` (`VolumeTearingEngine.inl:389-439`): `Coord vec_n;` (line 399) is never assigned
  — the principal-stress-direction computation that should feed it is commented out
  (`// TODO restore that`, line 398, and lines 402-410). Every plane computed downstream
  (`vec_P1M`/`vec_P2M`, lines 419-434) is built from this zero vector.
- `cutting()` (`VolumeTearingEngine.inl:483+`): the plane-position computation from
  `tetrahedronInfo->principalStressDirection1/2/3` is commented out (lines 511, 527-538,
  `// TODO restore that`, line 514).
- `cutting()`'s guard `if (indexTetraMaxStress < m_topology->getNbTetrahedra()+1 || choice)`
  (line 496) is always true when `indexTetraMaxStress` is left at its signed `-1` default (since
  `-1 < anything-non-negative`), so with the detection loop disabled, this branch still runs and
  calls `m_topology->getTetrahedron(indexTetraMaxStress)` (line 510) with `indexTetraMaxStress == -1`
  — implicit-converted to whatever unsigned index type `getTetrahedron()` expects, i.e. a huge
  out-of-bounds value. Worth guarding explicitly (`indexTetraMaxStress == -1` / `InvalidID` check)
  regardless of when the detection loop gets restored, so this doesn't silently crash the moment
  someone re-enables `choice` without also finishing §1.3's detection logic.

**Task:** track as one item — restoring/finishing volume tearing's stress detection and plane
computation, not urgent relative to §1.1/§1.2 (surface tearing) but should be scoped as real,
sizeable remaining work rather than "almost done".

### 1.4 T-junction handling in `algoFracturePath()` can insert a path entry from more than one triangle
`TearingAlgorithms.inl:209-232` (and the mirrored block for the path's other end,
`TearingAlgorithms.inl:243-266`): loops over `triAEdge` (the triangles around an edge) and, for
*every* triangle where `sign > 0.0`, inserts a new `POINT` entry at the front of `topoPath_list`.
For a normal 2-triangle-manifold edge this should only match one side, but there's no `break` after
a match — unlike the structurally similar loop in `computeIntersectionNeighborTriangle`
(`TearingAlgorithms.inl:92-100`), which does `break` once found. If a near-degenerate/borderline dot
product lets both adjacent triangles satisfy `sign > 0.0`, this inserts two conflicting `POINT`
entries at the front while `rmFirstEdge` (a single bool) only tracks "at least one was inserted",
and the caller only removes one entry (`new_edges.erase(new_edges.begin())`,
`TearingAlgorithms.inl:282`) — a narrow but real correctness bug on borderline geometry.

**Task:** add `break` after the first qualifying triangle in both loops, matching the sibling
method's pattern, unless there's a reason multiple insertions are intentional (unclear from the
code as written).

## 2. C++ errors / compiler warnings

Real warnings from `cmake --build C:/projects/sofa-build --config RelWithDebInfo --target Tearing`
(clean incremental build, no errors — plugin does compile):

- **`TearingAlgorithms.inl:112,120`** — unreferenced parameters on both empty
  `computeFracturePath()` overloads (§1.2).
- **`TearingEngine.inl:44,45`** — `C4189`, `t_b_ok`/`t_c_ok` initialized but never referenced in
  `computeEndPointsNeighboringTriangles()` — dead locals, safe to delete.
- **`TriangleCuttingController.inl:649`** — `C4189`, `PTAS` (the points-to-add list) fetched in
  `draw()` but never used — only the triangles-to-add list gets drawn. Minor dead code in a debug
  visualization method, not a functional gap like §1.1/§1.2.
- **`TearingScenarioEngine.inl:159`** — `C4189`, unused local `topo`.
- **`TriangleCuttingController.inl:196`** — `C4100`, unreferenced parameter `edgesInTri`.
- **Signed/unsigned narrowing (`C4245`)**: `TearingAlgorithms.inl:151,175,483,582,650`,
  `TearingScenarioEngine.inl:79` (×2), `VolumeTearingEngine.inl:350` — `int` literals/values
  assigned into `Index`-typed locals/members. Mostly the `-1`-as-sentinel pattern; worth an explicit
  `InvalidID`/`sofa::InvalidID` instead of a raw `-1` literal at each site so intent is unambiguous
  and warning-free.
- **`BaseTearingEngine.inl:770`** — `C4018`, signed/unsigned mismatch in a `<` comparison; worth
  checking directly given how many of the surrounding warnings are IDs escaping their intended
  unsigned range.
- **`double`→`float` truncation (`C4305`)**: `BaseTearingEngine.inl:752,760,761,777`,
  `TriangleCuttingController.inl:676,677`, `TearingScenarioEngine.inl:177` — likely harmless
  (drawing/debug color or coordinate literals) but worth a pass to make literals `f`-suffixed where
  `float` is genuinely intended.
- **`size_t`→smaller-type narrowing (`C4267`)**: `BaseTearingEngine.inl:769`,
  `TriangleCuttingController.inl:397`, `TearingAlgorithms.inl:398,570,571`.
- **`TearingAlgorithms.inl:377`** — `C4456`, declaration of `t` hides a previous local declaration.
- **`TearingAlgorithms.inl:578,646`** — `C4996`, `TriangleSetGeometryAlgorithms::computeTriangleBarycoefs`
  deprecated since SOFA v25.06, scheduled for removal in v26.12. Two call sites; migrate to
  `computeTriangleBarycentricCoordinates` before it's removed upstream.

## 3. Optimizations / maintainability

### 3.1 Interleaved-pair encoding via flat `vector<Index>` in `computeSegmentMeshIntersection`
`TearingAlgorithms.inl:413-555` stores edge-endpoint pairs as a flat `candidateIndice` vector,
accessed via `candidateIndice[2*i]`/`candidateIndice[2*i+1]` throughout. Error-prone (easy to get an
odd/even index wrong when touching this code) and harder to read than
`vector<std::pair<Index,Index>>` or a small named struct. Same pattern recurs for
`candidateBarycoef`/`candidateCoordKmin` indexed by `i` in lockstep. Not urgent, but worth
considering if this method gets touched for §1.1/§1.4.

### 3.2 Self-acknowledged magic number for border-snap epsilon
`TearingAlgorithms.inl:697`:
```cpp
double epsilonBorderSnap = (double)snapingBorderValue / 210; // magic number (0.5 is max value and must not be reached, as threshold is compared to barycoord value)
```
The comment itself flags this as fragile. Worth naming the constant (e.g.
`constexpr double kBorderSnapDenominator = 210.0;`) with the reasoning from the comment attached, so
it doesn't get "simplified" to 200 (matching the sibling `epsilonSnap` calc one line above) by
someone who doesn't know why it's deliberately different.

### 3.3 Three self-documented temporary hacks awaiting upstream SOFA changes
`TearingAlgorithms.inl:201,235,280` — `// TODO: Temporary Fix. To be removed and changed when new
TriangleSubvidier are integrated in SOFA` / `// TODO: hack to be removed`. All three are part of the
same first/last-edge-to-point conversion workaround in `algoFracturePath()`. Grouped here as one
known-debt item since fixing one likely means revisiting all three together, and they're explicitly
blocked on an upstream SOFA change rather than something to fix unilaterally in this plugin.

### 3.4 Debug draw silently caps at 20000 subdividers
`TriangleCuttingController.inl:645-646`: `if (cpt == 20000) break;` inside `draw()`'s loop over
`m_subviders`, with no log/warning when the cap is hit. Low priority (debug visualization only,
not simulation-affecting), but per the "no silent caps" principle, a one-line `msg_warning` the
first time this triggers would avoid confusing a user wondering why part of a large cut isn't drawn.

## Positive notes (not tasks, but worth recording)

- `TriangleCuttingController`'s subdivider/point ownership is clean: every `new TriangleSubdivider(...)`
  goes into `m_subviders`, `clearBuffers()` deletes and clears the whole vector, and it has exactly
  one call site (`TriangleCuttingController.inl:422`) — no leak, no double-free risk, no stale-buffer
  reuse across cuts. `m_pointsToAdd` uses `shared_ptr<PointToAdd>`, so `.clear()` there is correct as
  written. This plugin's cutting controller does not repeat the raw-pointer ownership bugs found in
  MeshRefinement's `ARCHITECTURE_REVIEW.md`.
