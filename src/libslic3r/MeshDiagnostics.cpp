#include "MeshDiagnostics.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>

namespace Slic3r {

namespace {

struct VertexFan {
    std::vector<size_t> faces;
    std::vector<size_t> parent;
};

struct EdgeRef {
    size_t v0;
    size_t v1;
    size_t v0_fan_idx;
    size_t v1_fan_idx;
};

// Compact record used by its_topology_stats(). The top bit of face_and_dir
// stores whether the directed face edge runs from the lower to the higher
// vertex index. Face indices fit in the remaining 31 bits because the legacy
// face-neighbor representation stores them in signed ints as well.
struct TopologyEdgeRef {
    uint32_t v0;
    uint32_t v1;
    uint32_t face_and_dir;
};

static_assert(sizeof(TopologyEdgeRef) == 12, "Keep topology records compact");

static constexpr uint32_t topology_direction_bit = uint32_t(1) << 31;

// Stable LSD radix sorting preserves the original ascending face order inside
// equal edge groups. That is required by the legacy greedy face-pairing rule,
// while avoiding millions of branch-heavy edge comparisons on large meshes.
static void topology_parallel_radix_sort(std::vector<TopologyEdgeRef> &edges)
{
    constexpr size_t radix_bits = 16;
    constexpr size_t radix_size = size_t(1) << radix_bits;
    constexpr size_t radix_mask = radix_size - 1;
    constexpr size_t pass_count = 64 / radix_bits;

    const size_t concurrency = std::max<size_t>(1, tbb::this_task_arena::max_concurrency());
    const size_t block_count = std::min(concurrency, std::max<size_t>(1, edges.size() / 262'144));

    std::vector<TopologyEdgeRef> scratch(edges.size());
    std::vector<size_t> counts(block_count * radix_size);
    std::vector<size_t> offsets(block_count * radix_size);
    TopologyEdgeRef *src = edges.data();
    TopologyEdgeRef *dst = scratch.data();

    for (size_t pass = 0; pass < pass_count; ++pass) {
        std::fill(counts.begin(), counts.end(), size_t(0));
        const unsigned component_shift = static_cast<unsigned>((pass & 1) * radix_bits);
        const bool     use_v0          = pass >= 2;

        tbb::parallel_for(size_t(0), block_count, [&](size_t block_idx) {
            size_t *local_counts = counts.data() + block_idx * radix_size;
            const size_t begin = edges.size() * block_idx / block_count;
            const size_t end   = edges.size() * (block_idx + 1) / block_count;
            for (size_t i = begin; i < end; ++i) {
                const uint32_t component = use_v0 ? src[i].v0 : src[i].v1;
                ++local_counts[(component >> component_shift) & radix_mask];
            }
        });

        size_t bucket_begin = 0;
        for (size_t bucket = 0; bucket < radix_size; ++bucket) {
            size_t block_begin = bucket_begin;
            for (size_t block_idx = 0; block_idx < block_count; ++block_idx) {
                const size_t idx = block_idx * radix_size + bucket;
                offsets[idx] = block_begin;
                block_begin += counts[idx];
            }
            bucket_begin = block_begin;
        }

        tbb::parallel_for(size_t(0), block_count, [&](size_t block_idx) {
            size_t *local_offsets = offsets.data() + block_idx * radix_size;
            const size_t begin = edges.size() * block_idx / block_count;
            const size_t end   = edges.size() * (block_idx + 1) / block_count;
            for (size_t i = begin; i < end; ++i) {
                const uint32_t component = use_v0 ? src[i].v0 : src[i].v1;
                dst[local_offsets[(component >> component_shift) & radix_mask]++] = src[i];
            }
        });

        std::swap(src, dst);
    }
}

static uint32_t topology_root(std::vector<uint32_t> &parent, uint32_t idx)
{
    uint32_t root = idx;
    while (parent[root] != root)
        root = parent[root];

    while (parent[idx] != idx) {
        const uint32_t next = parent[idx];
        parent[idx] = root;
        idx = next;
    }

    return root;
}

static bool topology_union(std::vector<uint32_t> &parent, uint32_t a, uint32_t b)
{
    const uint32_t root_a = topology_root(parent, a);
    const uint32_t root_b = topology_root(parent, b);
    if (root_a == root_b)
        return false;

    parent[root_b] = root_a;
    return true;
}

static size_t fan_root(std::vector<size_t> &parent, size_t idx)
{
    size_t root = idx;
    while (parent[root] != root)
        root = parent[root];

    while (parent[idx] != idx) {
        size_t next = parent[idx];
        parent[idx] = root;
        idx = next;
    }

    return root;
}

static void fan_union(VertexFan &fan, size_t a, size_t b)
{
    size_t root_a = fan_root(fan.parent, a);
    size_t root_b = fan_root(fan.parent, b);
    if (root_a != root_b)
        fan.parent[root_b] = root_a;
}

} // anonymous namespace

MeshDiagnosticStats its_mesh_diagnostics(const indexed_triangle_set &its)
{
    MeshDiagnosticStats result;
    const size_t     num_vertices = its.vertices.size();
    const size_t     num_faces    = its.indices.size();

    if (num_faces == 0)
        return result;

    // --- Pass 1: build per-vertex face fans and flat edge refs --------------
    std::vector<VertexFan> vertex_fans(num_vertices);
    std::vector<EdgeRef>   edge_refs;
    edge_refs.reserve(num_faces * 3);

    for (size_t fid = 0; fid < num_faces; ++fid) {
        const auto &face = its.indices[fid];

        // Skip degenerate faces (two or more identical vertex indices).
        if (face[0] == face[1] || face[1] == face[2] || face[2] == face[0])
            continue;

        size_t vertices[3] = {
            static_cast<size_t>(face[0]),
            static_cast<size_t>(face[1]),
            static_cast<size_t>(face[2])
        };
        size_t fan_indices[3] = { size_t(-1), size_t(-1), size_t(-1) };

        for (int i = 0; i < 3; ++i) {
            const size_t vid = vertices[i];
            if (vid >= num_vertices)
                continue;

            fan_indices[i] = vertex_fans[vid].faces.size();
            vertex_fans[vid].faces.push_back(fid);
        }

        for (int i = 0; i < 3; ++i) {
            const int    j  = (i + 1) % 3;
            const size_t va = vertices[i];
            const size_t vb = vertices[j];

            if (va >= num_vertices || vb >= num_vertices)
                continue;

            if (va < vb)
                edge_refs.push_back({ va, vb, fan_indices[i], fan_indices[j] });
            else
                edge_refs.push_back({ vb, va, fan_indices[j], fan_indices[i] });
        }
    }

    // Initialize per-vertex union-find storage. Non-degenerate faces insert
    // each incident face only once per vertex, so no sort/unique pass is needed.
    for (auto &fan : vertex_fans) {
        fan.parent.resize(fan.faces.size());
        std::iota(fan.parent.begin(), fan.parent.end(), 0);
    }

    // --- Edge classification (each undirected edge counted at most once) -----
    // Also mark vertices incident on non-manifold edges so that the vertex
    // fan-connectivity test below can skip them (same strategy as VCGlib).
    // Two-face edge groups connect those two face fans at both edge endpoints.
    std::vector<bool> on_nm_edge(num_vertices, false);

    std::sort(edge_refs.begin(), edge_refs.end(), [](const EdgeRef &a, const EdgeRef &b) {
        return a.v0 < b.v0 || (a.v0 == b.v0 && a.v1 < b.v1);
    });

    for (size_t i = 0; i < edge_refs.size();) {
        size_t j = i + 1;
        while (j < edge_refs.size() && edge_refs[j].v0 == edge_refs[i].v0 && edge_refs[j].v1 == edge_refs[i].v1)
            ++j;

        const size_t edge_face_count = j - i;
        if (edge_face_count == 1) {
            ++result.open_edges;
        } else if (edge_face_count == 2) {
            fan_union(vertex_fans[edge_refs[i].v0], edge_refs[i].v0_fan_idx, edge_refs[i + 1].v0_fan_idx);
            fan_union(vertex_fans[edge_refs[i].v1], edge_refs[i].v1_fan_idx, edge_refs[i + 1].v1_fan_idx);
        } else {
            ++result.non_manifold_edges;
            on_nm_edge[edge_refs[i].v0] = true;
            on_nm_edge[edge_refs[i].v1] = true;
        }

        i = j;
    }

    // --- Pass 2: non-manifold vertex detection ------------------------------
    // A vertex is non-manifold if its incident faces form more than one
    // component when connected through regular two-face edges.
    // Vertices on non-manifold edges are excluded: an edge with >2 faces
    // cannot define a reliable two-face fan traversal, and these vertices are
    // already accounted for by non_manifold_edges.
    std::vector<size_t> roots;

    for (size_t vid = 0; vid < num_vertices; ++vid) {
        if (on_nm_edge[vid])
            continue;

        auto &fan = vertex_fans[vid];
        if (fan.faces.size() <= 1)
            continue;

        roots.clear();
        roots.reserve(fan.parent.size());
        for (size_t i = 0; i < fan.parent.size(); ++i)
            roots.push_back(fan_root(fan.parent, i));

        std::sort(roots.begin(), roots.end());
        if (std::unique(roots.begin(), roots.end()) != roots.begin() + 1)
            ++result.non_manifold_vertices;
    }

    return result;
}

MeshDiagnosticStats its_edge_diagnostics(const indexed_triangle_set &its)
{
    MeshDiagnosticStats result;
    const size_t num_vertices = its.vertices.size();
    const size_t num_faces    = its.indices.size();

    if (num_faces == 0)
        return result;

    std::vector<std::pair<size_t, size_t>> edges;
    edges.reserve(num_faces * 3);

    for (size_t fid = 0; fid < num_faces; ++fid) {
        const auto &face = its.indices[fid];

        if (face[0] == face[1] || face[1] == face[2] || face[2] == face[0])
            continue;

        size_t v[3] = {
            static_cast<size_t>(face[0]),
            static_cast<size_t>(face[1]),
            static_cast<size_t>(face[2])
        };

        for (int i = 0; i < 3; ++i) {
            size_t va = v[i], vb = v[(i + 1) % 3];
            if (va >= num_vertices || vb >= num_vertices)
                continue;
            if (va > vb)
                std::swap(va, vb);
            edges.emplace_back(va, vb);
        }
    }

    if (edges.size() >= 1'000'000)
        tbb::parallel_sort(edges.begin(), edges.end());
    else
        std::sort(edges.begin(), edges.end());

    for (size_t i = 0; i < edges.size();) {
        size_t j = i + 1;
        while (j < edges.size() && edges[j] == edges[i])
            ++j;

        const size_t count = j - i;
        if (count == 1)
            ++result.open_edges;
        else if (count > 2)
            ++result.non_manifold_edges;

        i = j;
    }

    return result;
}

bool its_topology_stats(const indexed_triangle_set &its, MeshTopologyStats &result)
{
    result = {};

    const size_t num_vertices = its.vertices.size();
    const size_t num_faces    = its.indices.size();
    if (num_faces == 0)
        return true;

    if (num_faces > std::numeric_limits<int32_t>::max() ||
        num_faces > std::numeric_limits<size_t>::max() / 3)
        return false;

    std::vector<TopologyEdgeRef> edges(num_faces * 3);
    for (size_t face_idx = 0; face_idx < num_faces; ++face_idx) {
        const auto &face = its.indices[face_idx];
        if (face[0] < 0 || face[1] < 0 || face[2] < 0 ||
            static_cast<size_t>(face[0]) >= num_vertices ||
            static_cast<size_t>(face[1]) >= num_vertices ||
            static_cast<size_t>(face[2]) >= num_vertices ||
            face[0] == face[1] || face[1] == face[2] || face[2] == face[0])
            return false;

        for (int edge_idx = 0; edge_idx < 3; ++edge_idx) {
            const uint32_t va = static_cast<uint32_t>(face[edge_idx]);
            const uint32_t vb = static_cast<uint32_t>(face[(edge_idx + 1) % 3]);
            const bool     forward = va < vb;
            edges[face_idx * 3 + edge_idx] = {
                std::min(va, vb),
                std::max(va, vb),
                static_cast<uint32_t>(face_idx) | (forward ? topology_direction_bit : 0)
            };
        }
    }

    const auto edge_less = [](const TopologyEdgeRef &a, const TopologyEdgeRef &b) {
        if (a.v0 != b.v0)
            return a.v0 < b.v0;
        if (a.v1 != b.v1)
            return a.v1 < b.v1;
        return (a.face_and_dir & ~topology_direction_bit) < (b.face_and_dir & ~topology_direction_bit);
    };

    if (edges.size() >= 1'000'000)
        topology_parallel_radix_sort(edges);
    else
        std::sort(edges.begin(), edges.end(), edge_less);

    std::vector<uint32_t> face_parent(num_faces);
    std::iota(face_parent.begin(), face_parent.end(), uint32_t(0));
    result.number_of_parts = num_faces;

    for (size_t group_begin = 0; group_begin < edges.size();) {
        size_t group_end = group_begin + 1;
        while (group_end < edges.size() &&
               edges[group_end].v0 == edges[group_begin].v0 &&
               edges[group_end].v1 == edges[group_begin].v1)
            ++group_end;

        const size_t face_count = group_end - group_begin;
        if (face_count == 1)
            ++result.edge_stats.open_edges;
        else if (face_count > 2)
            ++result.edge_stats.non_manifold_edges;

        // Match create_face_neighbors_index(): visit faces in ascending order
        // and greedily pair each edge with the first later edge whose winding
        // is opposite. Each sorted edge group is independent.
        for (size_t i = group_begin; i < group_end; ++i) {
            if (edges[i].face_and_dir == std::numeric_limits<uint32_t>::max())
                continue;

            const bool direction = (edges[i].face_and_dir & topology_direction_bit) != 0;
            for (size_t j = i + 1; j < group_end; ++j) {
                if (edges[j].face_and_dir == std::numeric_limits<uint32_t>::max() ||
                    ((edges[j].face_and_dir & topology_direction_bit) != 0) == direction)
                    continue;

                const uint32_t face_a = edges[i].face_and_dir & ~topology_direction_bit;
                const uint32_t face_b = edges[j].face_and_dir & ~topology_direction_bit;
                if (topology_union(face_parent, face_a, face_b))
                    --result.number_of_parts;
                edges[i].face_and_dir = std::numeric_limits<uint32_t>::max();
                edges[j].face_and_dir = std::numeric_limits<uint32_t>::max();
                break;
            }
        }

        group_begin = group_end;
    }

    return true;
}


} // namespace Slic3r
