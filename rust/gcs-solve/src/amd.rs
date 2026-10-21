//! Approximate minimum degree (Amestoy, Davis & Duff), the fill-reducing order of a symmetric
//! pattern, as Davis writes it (`cs_amd`, *Direct Methods for Sparse Linear Systems*, ch. 7).
//!
//! The elimination is simulated on a quotient graph: an eliminated node becomes an *element*
//! whose list stands for the clique its elimination makes, so the graph never grows past A's
//! own pattern (plus elbow room for the new elements' lists, garbage-collected in place).  Each
//! step takes a node of least approximate degree — an upper bound on its true external degree,
//! got from the set differences |Le \ Lk| in time proportional to the pattern — and merges
//! nodes found indistinguishable (equal adjacency, by hash) into supervariables, eliminated
//! together.  A node touching more than `sparse::dense_limit` others is *dense*: left out of
//! the search and ordered last, as `sparse::Ata` orders a dense column.  The result is a
//! postorder of the assembly tree.
//!
//! On the meshes of a solid this is the order that keeps the factor near `n log n`, where a
//! banded order (`sparse`'s reverse Cuthill–McKee) costs `n^{5/3}` in 3D: what P1 of #510 is.
//!
//! The indices are signed throughout, as in the original: a negative entry is a flag
//! (`flip(i) = −i − 2`), and the code reads best kept beside the book's.

fn flip(i: isize) -> isize {
    -i - 2
}

/// Reset the marks in `w` when `mark + lemax` would pass what an `isize` holds.  On return every
/// live `w[k] < mark`.
fn wclear(mut mark: isize, lemax: isize, w: &mut [isize], n: usize) -> isize {
    if mark < 2 || mark.checked_add(lemax).is_none() {
        for k in 0..n {
            if w[k] != 0 {
                w[k] = 1;
            }
        }
        mark = 2;
    }
    mark
}

/// Depth-first postorder of the tree rooted at `j`, from its child lists (`head`, `next`),
/// written to `post` from `k`.  Returns the next free position.
fn tdfs(
    j: isize,
    mut k: usize,
    head: &mut [isize],
    next: &[isize],
    post: &mut [isize],
    stack: &mut [isize],
) -> usize {
    let mut top: isize = 0;
    stack[0] = j;
    while top >= 0 {
        let p = stack[top as usize];
        let i = head[p as usize];
        if i == -1 {
            top -= 1;
            post[k] = p;
            k += 1;
        } else {
            head[p as usize] = next[i as usize];
            top += 1;
            stack[top as usize] = i;
        }
    }
    k
}

/// The AMD order of the symmetric pattern given by column: `ap[j]..ap[j + 1]` index the rows of
/// column `j` in `ai`, both triangles, the diagonal allowed (it is dropped).  `perm[k]` is the
/// column eliminated k-th.
pub fn order(n: usize, ap: &[usize], ai: &[usize]) -> Vec<usize> {
    if n == 0 {
        return Vec::new();
    }
    // C: the pattern without its diagonal, with elbow room for the elements' lists
    let mut cp = vec![0isize; n + 1];
    let mut ci: Vec<isize> = Vec::with_capacity(ap[n] + ap[n] / 5 + 2 * n);
    for j in 0..n {
        for p in ap[j]..ap[j + 1] {
            if ai[p] != j {
                ci.push(ai[p] as isize);
            }
        }
        cp[j + 1] = ci.len() as isize;
    }
    let mut cnz = ci.len() as isize;
    let nzmax = cnz + cnz / 5 + 2 * n as isize;
    ci.resize(nzmax as usize, 0);

    let ni = n as isize;
    let dense = crate::sparse::dense_limit(n);
    let mut p_out = vec![0isize; n + 1];
    let mut len = vec![0isize; n + 1];
    let mut nv = vec![1isize; n + 1];
    let mut next = vec![-1isize; n + 1];
    let mut head = vec![-1isize; n + 1];
    let mut elen = vec![0isize; n + 1];
    let mut degree = vec![0isize; n + 1];
    let mut w = vec![1isize; n + 1];
    let mut hhead = vec![-1isize; n + 1];
    // `last` shares the output's storage in the original; kept apart here
    let mut last = vec![-1isize; n + 1];

    for k in 0..n {
        len[k] = cp[k + 1] - cp[k];
        degree[k] = len[k];
    }
    len[n] = 0;
    degree[n] = 0;
    let mut mark = wclear(0, 0, &mut w, n);
    elen[n] = -2; // n is a dead element: the root dense nodes are absorbed into
    cp[n] = -1;
    w[n] = 0;
    let mut nel: isize = 0;
    let mut lemax: isize = 0;
    let mut mindeg: isize = 0;

    // degree lists
    for i in 0..n {
        let d = degree[i];
        if d == 0 {
            elen[i] = -2;
            nel += 1;
            cp[i] = -1;
            w[i] = 0;
        } else if d as f64 > dense {
            nv[i] = 0;
            elen[i] = -1;
            nel += 1;
            cp[i] = flip(ni);
            nv[n] += 1;
        } else {
            let du = d as usize;
            if head[du] != -1 {
                last[head[du] as usize] = i as isize;
            }
            next[i] = head[du];
            head[du] = i as isize;
        }
    }

    while nel < ni {
        // --- the node of least approximate degree
        let mut k: isize = -1;
        while mindeg < ni {
            k = head[mindeg as usize];
            if k != -1 {
                break;
            }
            mindeg += 1;
        }
        let ku = k as usize;
        if next[ku] != -1 {
            last[next[ku] as usize] = -1;
        }
        head[mindeg as usize] = next[ku];
        let elenk = elen[ku];
        let mut nvk = nv[ku];
        nel += nvk;

        // --- garbage collection
        if elenk > 0 && cnz + mindeg >= nzmax {
            for j in 0..n {
                let p = cp[j];
                if p >= 0 {
                    cp[j] = ci[p as usize];
                    ci[p as usize] = flip(j as isize);
                }
            }
            let (mut q, mut p) = (0isize, 0isize);
            while p < cnz {
                let j = flip(ci[p as usize]);
                p += 1;
                if j >= 0 {
                    let ju = j as usize;
                    ci[q as usize] = cp[ju];
                    cp[ju] = q;
                    q += 1;
                    for _ in 0..len[ju] - 1 {
                        ci[q as usize] = ci[p as usize];
                        q += 1;
                        p += 1;
                    }
                }
            }
            cnz = q;
        }

        // --- the new element Lk
        let mut dk: isize = 0;
        nv[ku] = -nvk;
        let mut p = cp[ku];
        let pk1 = if elenk == 0 { p } else { cnz };
        let mut pk2 = pk1;
        for k1 in 1..=elenk + 1 {
            let (e, mut pj, ln);
            if k1 > elenk {
                e = k;
                pj = p;
                ln = len[ku] - elenk;
            } else {
                e = ci[p as usize];
                p += 1;
                pj = cp[e as usize];
                ln = len[e as usize];
            }
            for _ in 1..=ln {
                let i = ci[pj as usize];
                pj += 1;
                let iu = i as usize;
                let nvi = nv[iu];
                if nvi <= 0 {
                    continue;
                }
                dk += nvi;
                nv[iu] = -nvi;
                ci[pk2 as usize] = i;
                pk2 += 1;
                if next[iu] != -1 {
                    last[next[iu] as usize] = last[iu];
                }
                if last[iu] != -1 {
                    next[last[iu] as usize] = next[iu];
                } else {
                    head[degree[iu] as usize] = next[iu];
                }
            }
            if e != k {
                cp[e as usize] = flip(k);
                w[e as usize] = 0;
            }
        }
        if elenk != 0 {
            cnz = pk2;
        }
        degree[ku] = dk;
        cp[ku] = pk1;
        len[ku] = pk2 - pk1;
        elen[ku] = -2;

        // --- set differences |Le \ Lk|
        mark = wclear(mark, lemax, &mut w, n);
        for pk in pk1..pk2 {
            let i = ci[pk as usize] as usize;
            let eln = elen[i];
            if eln <= 0 {
                continue;
            }
            let nvi = -nv[i];
            let wnvi = mark - nvi;
            for p in cp[i]..cp[i] + eln {
                let e = ci[p as usize] as usize;
                if w[e] >= mark {
                    w[e] -= nvi;
                } else if w[e] != 0 {
                    w[e] = degree[e] + wnvi;
                }
            }
        }

        // --- degree update
        for pk in pk1..pk2 {
            let i = ci[pk as usize] as usize;
            let p1 = cp[i];
            let p2 = p1 + elen[i] - 1;
            let mut pn = p1;
            let mut h: isize = 0;
            let mut d: isize = 0;
            for p in p1..=p2 {
                let e = ci[p as usize];
                let eu = e as usize;
                if w[eu] != 0 {
                    let dext = w[eu] - mark;
                    if dext > 0 {
                        d += dext;
                        ci[pn as usize] = e;
                        pn += 1;
                        h = h.wrapping_add(e);
                    } else {
                        cp[eu] = flip(k); // aggressive absorption
                        w[eu] = 0;
                    }
                }
            }
            elen[i] = pn - p1 + 1;
            let p3 = pn;
            let p4 = p1 + len[i];
            for p in p2 + 1..p4 {
                let j = ci[p as usize];
                let nvj = nv[j as usize];
                if nvj <= 0 {
                    continue;
                }
                d += nvj;
                ci[pn as usize] = j;
                pn += 1;
                h = h.wrapping_add(j);
            }
            if d == 0 {
                // mass elimination
                cp[i] = flip(k);
                let nvi = -nv[i];
                dk -= nvi;
                nvk += nvi;
                nel += nvi;
                nv[i] = 0;
                elen[i] = -1;
            } else {
                degree[i] = degree[i].min(d);
                ci[pn as usize] = ci[p3 as usize];
                ci[p3 as usize] = ci[p1 as usize];
                ci[p1 as usize] = k;
                len[i] = pn - p1 + 1;
                let hu = (h.unsigned_abs() % n) as isize;
                next[i] = hhead[hu as usize];
                hhead[hu as usize] = i as isize;
                last[i] = hu;
            }
        }
        degree[ku] = dk;
        lemax = lemax.max(dk);
        mark = wclear(mark + lemax, lemax, &mut w, n);

        // --- supervariables: nodes of Lk with the same adjacency
        for pk in pk1..pk2 {
            let i0 = ci[pk as usize] as usize;
            if nv[i0] >= 0 {
                continue;
            }
            let h = last[i0] as usize;
            let mut i = hhead[h];
            hhead[h] = -1;
            while i != -1 && next[i as usize] != -1 {
                let iu = i as usize;
                let ln = len[iu];
                let eln = elen[iu];
                for p in cp[iu] + 1..cp[iu] + ln {
                    w[ci[p as usize] as usize] = mark;
                }
                let mut jlast = i;
                let mut j = next[iu];
                while j != -1 {
                    let ju = j as usize;
                    let mut ok = len[ju] == ln && elen[ju] == eln;
                    let mut p = cp[ju] + 1;
                    while ok && p < cp[ju] + ln {
                        if w[ci[p as usize] as usize] != mark {
                            ok = false;
                        }
                        p += 1;
                    }
                    if ok {
                        cp[ju] = flip(i);
                        nv[iu] += nv[ju];
                        nv[ju] = 0;
                        elen[ju] = -1;
                        j = next[ju];
                        next[jlast as usize] = j;
                    } else {
                        jlast = j;
                        j = next[ju];
                    }
                }
                i = next[iu];
                mark += 1;
            }
        }

        // --- finalise Lk
        let mut p = pk1;
        for pk in pk1..pk2 {
            let i = ci[pk as usize];
            let iu = i as usize;
            let nvi = -nv[iu];
            if nvi <= 0 {
                continue;
            }
            nv[iu] = nvi;
            let d = (degree[iu] + dk - nvi).min(ni - nel - nvi);
            let du = d as usize;
            if head[du] != -1 {
                last[head[du] as usize] = i;
            }
            next[iu] = head[du];
            last[iu] = -1;
            head[du] = i;
            mindeg = mindeg.min(d);
            degree[iu] = d;
            ci[p as usize] = i;
            p += 1;
        }
        nv[ku] = nvk;
        len[ku] = p - pk1;
        if len[ku] == 0 {
            cp[ku] = -1;
            w[ku] = 0;
        }
        if elenk != 0 {
            cnz = p;
        }
    }

    // --- postorder the assembly tree
    for i in 0..n {
        cp[i] = flip(cp[i]);
    }
    for j in 0..=n {
        head[j] = -1;
    }
    for j in (0..=n).rev() {
        if nv[j] > 0 {
            continue; // an element
        }
        let parent = cp[j] as usize;
        next[j] = head[parent];
        head[parent] = j as isize;
    }
    for e in (0..=n).rev() {
        if nv[e] <= 0 || cp[e] == -1 {
            continue;
        }
        let parent = cp[e] as usize;
        next[e] = head[parent];
        head[parent] = e as isize;
    }
    let mut k = 0usize;
    for i in 0..=n {
        if cp[i] == -1 {
            k = tdfs(i as isize, k, &mut head, &next, &mut p_out, &mut w);
        }
    }
    debug_assert_eq!(k, n + 1);
    // the root `n` is postordered last
    debug_assert_eq!(p_out[n], ni);
    p_out[..n].iter().map(|&i| i as usize).collect()
}
