# One way to define a name: plan and record

**Status (2026-10-02): implemented** on branch `walrus-definitions`, except a name in a child slot
(`line(a, q := hint(…))`, `solid(f := face(…), …)`), which does not parse yet. The corpus, the
tests' inline documents, the primer, the spec ([0.29]) and CLAUDE.md are rewritten. Every
example's report and SVG is byte-identical to before but for absolute paths, error columns in
module-only files, and anonymous instance keys (`#i…`, offsets) in `data-path` attributes. One
decision was added in the doing: a drawn callout prints a definition `w = 60` (`io::as_written`).

Solvent has accreted several spellings for putting a name into scope: `param w = 100`, a named
dimension's `w = 60`, an instance's `cyl: Cylinder(…)`, a declaration's `circle c(…)`, a chain's
`profile = …`, a computed point's `point p = (…)`, a curve's `curve k = …`, a group's
`group dims(…)`. They do one thing. This plan makes **`name := expression`** the only way to
define a name, and makes it an expression: `(name := e)` is `e`, with a name on it.

Nothing has been deployed, so there is no compatibility: the old spellings stop parsing, the
corpus is rewritten in the same change, and no diagnostic mentions an old spelling.

## The rule

Two punctuations, two acts, and no third:

* **`:=` defines.** It puts a name into the enclosing scope: the body of the file, a component,
  a block copy. Whatever stands on its right is the value, and the name is that value.
* **`label:` fills.** It fills a slot of the thing being called or declared: a call's argument
  (`dir: dir`), a constructor's child (`center: o`), a `hint(…)` key (`r: 25`), a group's member
  (`width: 20mm`), a component's formal (`r: Length`). None of these puts a name into the
  caller's scope. A formal does put one into the component's own, but it is the declaration of a
  slot, and a caller fills it with the same `label:`, so the two sides of a call are spelled
  alike.

The binders stay as they are: `repeat N as i`, `repeat e in CHAIN`, `over u in (a, b)`,
`claim over θ in (…)`. They bind a name over a block or an interval rather than to a value. So
does `component Name(…) { … }`: a component is not a value, so it is not passed, returned or
named by `:=`, and a spelling that suggested it was would be wrong. `use`, `unit` and `style`
define nothing.

## Before and after

| Before | After |
|---|---|
| `param w = 100` | `w := 100` |
| `a distance(w = 60) b` | `a distance(w := 60) b` |
| `cyl: Cylinder(…)` | `cyl := Cylinder(…)` |
| `t: Tooth(…) in top` | `t := Tooth(…) in top` |
| `private construction layout: Polygon(…)` | `private construction layout := Polygon(…)` |
| `circle c(center: o) hint(r: 25)` | `c := circle(center: o) hint(r: 25)` |
| `construction centerline line spindle(a, b)` | `construction centerline spindle := line(a, b)` |
| `point p = (o.x + u, o.y)` | `p := point(x: o.x + u, y: o.y)` |
| `plane f(origin: o, toward: q)` | `f := plane(origin: o, toward: q)` |
| `solid body(stock)` | `body := solid(stock)` |
| `motion turn(about: axis, ratio: 2)` | `turn := motion(about: axis, ratio: 2)` |
| `profile = line ab(a, b) -> line -> close` | `profile := (ab := line(a, b)) -> line -> close` |
| `curve k = leg.toe over theta in (0, 360)` | `k := leg.toe over theta in (0, 360)` |
| `group dims(width: 20mm, origin: o)` | `dims := group(width: 20mm, origin: o)` |

Anonymous forms are unchanged: `line(a, b)`, `circle hint(r: 25)`, `line -> line -> close`,
`Tooth(…)`. Every spatial kind (`surface`, `envelope`, `patch`, `seam`, `vertex`, `edge`, `face`,
`sphere`, `cone`, `cylinder`) follows the `solid` row: the kind keyword is the head of the
value, and its brackets are what it is made of.

## Decisions

**Modifiers stand before the name.** `private` is about the name and `construction` /
`centerline` about the geometry, but both are written where they are today, ahead of
everything: `[private] [construction] [centerline] NAME := EXPR`, and `[construction] EXPR` for
an anonymous statement. Inside parentheses (`(private ab := line)`) they are allowed with the
same meaning.

**`:=` binds loosest.** `profile := line -> line -> close` names the chain, and a link is named
in parentheses: `(ab := line(a, b)) -> line`. Inside an argument list no parentheses are needed,
since the commas delimit it and `:=` cannot be mistaken for `:`: `line(a, q := hint(x: 5, y: 0))`
and `arc(center: c := hint(x: 0, y: 0))` both declare the point they name.

**A prefix word's value is its operand.** `horizontal line(a, b)` states the constraint and its
value is the line, so `l := horizontal line(a, b)` names the line. A chain's value is a
traversal only once it has a joint. This is what makes the short form of the common statement
mean what it reads as; `horizontal (l := line(a, b))` says the same thing.

**A computed point's coordinates are its brackets.** `point(x: e, y: e)` is what the point is
made of; `point hint(x: e, y: e)` is where a solve starts. That is §4.3's rule for every other
kind, applied to the one that broke it with `= (…)`. A point has no children, so its brackets
were free. The sheet still refuses a computed point (nothing there holds a point to a formula).

**A curve needs no keyword.** `over FORMAL in (A, B)` marks the expression as a trace, so
`k := leg.toe over theta in (0, 360)` and `e := Ellipse(f, a: 40, b: 25).p over u in (0, 360)`
are curves.

**In a numeric slot `:=` stands outermost or not at all.** `a distance(w := 60) b` names a
dimension; `distance(2 * (w := 30))` is a parse error. A dimension is one node of the expression
graph and its callout draws its text; an inner name would be a param smuggled into a callout's
label. Statement-level `w := e` reads names but defines no second one inside `e`.

**Bare `=` is gone.** `:=` defines and `==` pins (`t == 0.4`), and the lexer has no `Tok::Eq`.
The `expr.rs` assignment form `w = 80` becomes `w := 80`. Equality in a claim or a report line
(`body.volume = 72000`) is output, not syntax, and is unaffected.

**What a statement-level definition is follows from its value, not its spelling.** `w := 100`
is a param: worked out while elaborating, never an unknown, never a seed, and refused (E107)
when it measures geometry. `c := circle(…)` is geometry; `cyl := Cylinder(…)` an instance;
`dims := group(…)` a group. A module still exports its top-level numbers and groups
(`module_params`) and not its geometry, which is what it exports today under `param` and
`group`.

**Unchanged:** a name nothing defines is a free variable (W111); one namespace for params and
named dimensions (E001 on a second definition, of either); built-in names refused or warned
(W112); a block copy's names indexed (`l[2]`); anonymous keys (`#a…`) never reaching the source;
`DeclName`'s three questions.

## What gets simpler

* **`P::declined` and the colon guards go, and `names_decl` survives only inside `is_name`.**
  A statement that begins `IDENT :=` is a definition after one token of lookahead. There is no longer a question of
  whether the word after `line` is the line's name or a trailing clause, so the reserved-word
  note (issue #33) has nothing left to explain; the guard that keeps `claim: Tooth(…)` an
  instance rather than a claim goes with it. Keywords still cannot be names, but that is a plain lexical rule.
* **`StmtKind::Param`, the instance arm, `NamedChain`'s `=` and the curve statement fold into one
  definition statement** whose right side is parsed as the expression it is. The kinds below
  the parser (`Decl`, `Instance`, `ParamDecl`, `CurveSpec`, `NamedChain`) can stay as they are
  in this change; only how they are reached changes.
* **The named dimension and the param are spelled alike**, which they already were in
  `flatten::params` (both `Def`s).

## What gets harder

**Minting a name is sometimes a wrap.** `edit::reconcile` mints by inserting at
`DeclName::span`, the empty span where a name would go. Under `:=` the insertion goes before the
expression: `NAME := ` at statement position, and `(NAME := ` … `)` where the declaration is a
chain link or an operand. The parser knows which at the point it parses the declaration, so it
records it beside the span (a `wrap: bool` on the key, or a second empty span at the
expression's end). The two insertions land at different offsets, so `splice`'s ordering only
has to place them relative to other edits at the same offsets, as it does today.

**Writers spell definitions in a new order.** `write_decl`, `syntax::print`, `add_entity`,
`add_point`, `mint`, `commit_seeds` (where it writes a whole argument list at the name's span,
the anchor moves), the app tools that append statements, and the colouring's `opens_link` and
`chain_starts` lookahead.

## Order of work

1. **Rewrite tool against the current parser.** One pass over every `.sv` in `rust/examples/`,
   `rust/lib/` and the test sources' inline documents, driven by spans the current parser
   already has (`Decl::name`, `hint_span`, `list_span`, the param and instance spans), never by
   regex. Kept in the scratchpad, not the repo.
2. **Parser and lexer switch outright.** `:=` in, `=` and the old forms out; `names_decl`,
   `declined` and the colon guards deleted. Lowered to the existing `StmtKind`s.
3. **Rewrite the corpus and the inline test documents** with the tool, in the same commit as
   step 2.
4. **Writers and minting** (above), with `tests/anonymous.rs` extended to mint a link inside a
   chain and a declaration standing as an operand.
5. **Check:** every example's SVG and `solventc` report byte-identical to before the change;
   `make test` once.
6. **Docs:** the primer (§1.3's statement forms first), `solvent-spec.md`, CLAUDE.md's
   convention entries that quote old spellings, and the app's insert tools' templates.

## Not in this change

* `.svd` names its models, sheets and views (`model part from "…"`, `sheet piston { … }`,
  `view front(…)`). The same rule would read `part := model("…")` and
  `front := view(part.body) from front at (…)`; it is a separate grammar, and follows once this
  lands.
* An explicit unknown (`w := free`) to tell a free variable from a typo.
* Removing the existing refusals for long-retired words (`frame`, `port`, `ring`, `ellipse`).
