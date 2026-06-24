#pragma once

#include <cstddef>
#include <string>
#include <vector>

// SMP XML "pattern" expander.
//
// A *pattern* is a reusable, parameterized macro for SMP physics XML. An author defines one once:
//
//     <pattern-default name="cape">
//       <param name="prefix"/>
//       <param name="rows" default="6"/>
//       <body>
//         <repeat var="i" count="${rows}">
//           <bone name="${prefix}_${i}"> ... </bone>
//         </repeat>
//         <repeat var="i" count="${rows}" from="1">
//           <generic-constraint bodyA="${prefix}_${i-1}" bodyB="${prefix}_${i}" template="link"/>
//         </repeat>
//       </body>
//     </pattern-default>
//
// and instantiates it where the real bones/constraints would go:
//
//     <pattern name="cape" prefix="NPC_Cape" rows="8"/>
//
// `expandPatterns` rewrites the document so every <pattern> use becomes the ordinary
// bone/constraint/etc. elements it stands for, and the <pattern-default> definitions are removed.
// Everything downstream (the runtime loader, XSD, Schematron, redundancy analysis) then sees only
// plain SMP elements and needs no knowledge that patterns exist. The five physics-XML parse sites
// each run this in front of their own parser so they all agree on the same expanded reality.

namespace hdt
{
	/// Severity of a single expansion diagnostic.
	enum class PatternDiagSeverity
	{
		Error,
		Warning
	};

	/// One problem found while expanding (undefined pattern, missing param, bad count, etc.).
	/// `line` is the 1-based line in the ORIGINAL (pre-expansion) text, or 0 when unknown.
	struct PatternDiag
	{
		PatternDiagSeverity severity = PatternDiagSeverity::Error;
		std::string message;
		int line = 0;
		std::string patternName;  ///< pattern involved, or empty
	};

	/// A half-open byte range [lo, hi) in the EXPANDED text that was produced by expanding one
	/// <pattern> use. Lets offset-reporting consumers (Schematron, redundancy) say "this came from
	/// pattern X used at line N" instead of pointing at generated text, and skip generated regions.
	struct PatternRange
	{
		std::size_t lo = 0;
		std::size_t hi = 0;
		std::string patternName;
		int useLine = 0;  ///< 1-based line of the <pattern> use in the original text
	};

	/// Ordered list of generated ranges plus a point-query.
	struct PatternSourceMap
	{
		std::vector<PatternRange> ranges;

		/// Returns the innermost range containing `offset`, or nullptr if `offset` is hand-written.
		/// Innermost wins so a pattern-inside-a-pattern attributes to the inner use.
		const PatternRange* find(std::size_t offset) const;
	};

	/// Tunable safety caps. XML is a trust boundary, so expansion must never hang or OOM on hostile
	/// or buggy input; every cap converts an unbounded blow-up into a clean Error diagnostic.
	struct PatternLimits
	{
		int maxRecursionDepth = 8;            ///< pattern-nested-in-pattern depth
		long maxRepeatCount = 10000;          ///< upper bound on a single <repeat count>
		std::size_t maxExpandedElements = 50000;  ///< total elements emitted across the document
	};

	/// A document of <pattern-default> definitions made available to every file's expansion -- e.g. one
	/// file from the global patterns/ folder. `origin` labels it for diagnostics and sets load order:
	/// when two libraries (or a library and the file) define the same pattern, the one applied later wins.
	struct PatternLibrary
	{
		std::string xml;
		std::string origin;
	};

	/// Everything expansion needs besides the document itself: the safety caps plus any shared pattern
	/// libraries whose definitions are visible in addition to the file's own <pattern-default>s.
	struct PatternOptions
	{
		PatternLimits limits;
		const std::vector<PatternLibrary>* libraries = nullptr;  ///< load order; later entries override earlier
	};

	/// Result of expansion. On success `xml` is the rewritten document; on failure (`ok == false`)
	/// `xml` is the original input unchanged and `diags` explains why -- callers must NOT feed a
	/// failed/half-expanded document to a parser (fail closed).
	struct PatternExpansion
	{
		std::string xml;
		bool ok = true;
		bool changed = false;  ///< true only when a pattern was actually expanded
		std::vector<PatternDiag> diags;
		PatternSourceMap sourceMap;
	};

	/// Expand <pattern>/<pattern-default> macros in an SMP physics XML document.
	///
	/// Algorithm:
	///   1. Short-circuit: if `raw` does not even contain the substring "<pattern", return it
	///      untouched with no parsing. The vast majority of existing files hit this path and stay
	///      byte-identical at zero cost.
	///   2. Parse `raw` with pugixml (full fidelity: comments, PIs, declaration preserved).
	///   3. Collect every <pattern-default> -- first from `options.libraries` (in load order), then from
	///      the file -- into a registry keyed by namespaced name (author.name) and version; a later
	///      definition of the same key overrides an earlier one (the file overrides a library).
	///   4. Rebuild the tree: copy ordinary nodes through, dropping <pattern-default> definitions and
	///      replacing each <pattern> use with its body -- with ${param} / ${loopVar} / ${loopVar±N}
	///      substituted and <repeat> loops unrolled (nested repeats give 2-D grids).
	///   5. Serialize the rebuilt tree back to text.
	///
	/// Fail closed: an undefined pattern, a missing required parameter, an unknown ${x}, a bad or
	/// over-cap <repeat count>, exceeding the recursion/element caps, or a parse failure all yield
	/// `ok == false` with `xml == raw`.
	PatternExpansion expandPatterns(const std::string& raw, const PatternOptions& options = {});

	/// Convenience overload with no shared libraries -- expands only the file's own patterns.
	PatternExpansion expandPatterns(const std::string& raw, const PatternLimits& limits);
}
