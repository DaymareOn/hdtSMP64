#include "hdtXmlPatternExpander.h"

#include <pugixml.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace hdt
{
	namespace
	{
		// ── Variable environment ────────────────────────────────────────────────
		// A value bound to a name during expansion. Pattern parameters are plain strings; <repeat>
		// loop variables additionally carry an integer so that ${i+1} arithmetic is possible.
		struct EnvVal
		{
			std::string str;
			bool isInt = false;
			long i = 0;
		};

		// Ordered stack of (name -> value) bindings. Lookups scan from the back so an inner <repeat>
		// shadows an outer binding of the same name. Kept as a vector because scopes are tiny.
		using Env = std::vector<std::pair<std::string, EnvVal>>;

		const EnvVal* lookup(const Env& env, const std::string& name)
		{
			for (auto it = env.rbegin(); it != env.rend(); ++it)
				if (it->first == name)
					return &it->second;
			return nullptr;
		}

		// A collected <pattern-default> definition: its declared parameters and its <body> template.
		struct ParamDecl
		{
			std::string name;
			bool hasDefault = false;
			std::string defVal;
		};

		struct PatternDef
		{
			std::vector<ParamDecl> params;
			pugi::xml_node body;               // points into the document (file or library) that declared it
			const std::string* src = nullptr;  // that document's text, for line numbers in body diagnostics
			std::string origin;                // "<file>" or a library origin, for conflict messages
		};

		// Shared mutable state for one expansion run. `aborted` latches on the first Error so the
		// recursion unwinds promptly and the caller discards the half-built tree (fail closed).
		struct Ctx
		{
			const std::string& raw;
			const PatternLimits& limits;
			// fullName (author.name) -> version ("" = unversioned) -> definition.
			std::map<std::string, std::map<std::string, PatternDef>> defs;
			std::vector<std::unique_ptr<pugi::xml_document>> libDocs;  ///< owns parsed library documents
			std::vector<PatternDiag> diags;
			std::vector<std::pair<std::string, int>> patUses;  ///< per <pattern> use: (name, source line)
			std::size_t elementCount = 0;
			bool aborted = false;
			bool changed = false;  ///< a <pattern> use was expanded
			bool sawAny = false;   ///< any real pattern element (def or use) was seen

			void error(int line, std::string msg, std::string pat = {})
			{
				diags.push_back({ PatternDiagSeverity::Error, std::move(msg), line, std::move(pat) });
				aborted = true;
			}
			void warn(int line, std::string msg, std::string pat = {})
			{
				diags.push_back({ PatternDiagSeverity::Warning, std::move(msg), line, std::move(pat) });
			}
		};

		// ── Small text helpers ──────────────────────────────────────────────────

		std::string trim(const std::string& s)
		{
			std::size_t a = 0, b = s.size();
			while (a < b && std::isspace(static_cast<unsigned char>(s[a])))
				++a;
			while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])))
				--b;
			return s.substr(a, b - a);
		}

		// Strict signed-integer parse: optional +/- then digits only, nothing else. Returns nullopt on
		// any deviation so "8x", "", "1.0" and overflow are all rejected rather than silently coerced.
		std::optional<long> parseLong(const std::string& s)
		{
			if (s.empty())
				return std::nullopt;
			std::size_t i = 0;
			if (s[i] == '+' || s[i] == '-')
				++i;
			if (i >= s.size())
				return std::nullopt;
			for (std::size_t j = i; j < s.size(); ++j)
				if (!std::isdigit(static_cast<unsigned char>(s[j])))
					return std::nullopt;
			try
			{
				return std::stol(s);
			}
			catch (...)
			{
				return std::nullopt;
			}
		}

		// 1-based line number of a byte offset in the original text (0 if the offset is unknown).
		int offsetToLine(const std::string& s, std::ptrdiff_t off)
		{
			if (off < 0 || static_cast<std::size_t>(off) > s.size())
				return 0;
			int line = 1;
			for (std::ptrdiff_t i = 0; i < off; ++i)
				if (s[i] == '\n')
					++line;
			return line;
		}

		// Offset just past the element whose opening '<' is at `start`, found by a name-agnostic tag
		// depth count (comments / CDATA / PIs skipped). Relies on the serializer escaping '<' and '>'
		// inside attribute values -- pugixml does -- so '<' and '>' only ever appear as real markup.
		// Used to delimit a pattern-generated subtree in the serialized output (see source-map below).
		std::size_t subtreeEnd(const std::string& s, std::size_t start)
		{
			std::size_t i = start;
			int depth = 0;
			while (i < s.size())
			{
				if (s[i] != '<')
				{
					++i;
					continue;
				}
				if (s.compare(i, 4, "<!--") == 0)
				{
					const std::size_t e = s.find("-->", i + 4);
					i = (e == std::string::npos ? s.size() : e + 3);
					continue;
				}
				if (s.compare(i, 9, "<![CDATA[") == 0)
				{
					const std::size_t e = s.find("]]>", i + 9);
					i = (e == std::string::npos ? s.size() : e + 3);
					continue;
				}
				if (s.compare(i, 2, "<?") == 0)
				{
					const std::size_t e = s.find("?>", i + 2);
					i = (e == std::string::npos ? s.size() : e + 2);
					continue;
				}
				if (s.compare(i, 2, "</") == 0)
				{
					const std::size_t e = s.find('>', i);
					i = (e == std::string::npos ? s.size() : e + 1);
					if (--depth <= 0)
						return i;
					continue;
				}
				const std::size_t e = s.find('>', i);
				if (e == std::string::npos)
					return s.size();
				const bool selfClose = (e > i && s[e - 1] == '/');
				i = e + 1;
				if (!selfClose)
					++depth;
				else if (depth == 0)
					return i;
			}
			return i;
		}

		// ── Substitution ────────────────────────────────────────────────────────

		// Resolves one ${...} body. Accepts either a bare name (param or loop var) or `name±N` index
		// arithmetic, where the base name MUST be an integer loop variable. On any failure it records
		// an Error and returns false.
		bool resolveExpr(Ctx& ctx, const std::string& expr, const Env& env, int line, std::string& out)
		{
			if (expr.empty() || !(std::isalpha(static_cast<unsigned char>(expr[0])) || expr[0] == '_'))
			{
				ctx.error(line, "bad ${} expression '" + expr + "'");
				return false;
			}
			std::size_t k = 0;
			while (k < expr.size() && (std::isalnum(static_cast<unsigned char>(expr[k])) || expr[k] == '_'))
				++k;
			const std::string name = expr.substr(0, k);
			const std::string rest = expr.substr(k);

			const EnvVal* v = lookup(env, name);
			if (!v)
			{
				ctx.error(line, "unknown variable '${" + name + "}'");
				return false;
			}
			if (rest.empty())
			{
				out = v->str;
				return true;
			}
			// rest must be (+|-)digits. The base must be an integer: a loop variable always is; a
			// parameter qualifies only if its value parses as one (so a bound like count="${rows-1}" works).
			if (rest.size() < 2 || (rest[0] != '+' && rest[0] != '-'))
			{
				ctx.error(line, "bad index expression '${" + expr + "}'");
				return false;
			}
			long base = 0;
			if (v->isInt)
			{
				base = v->i;
			}
			else
			{
				const std::optional<long> b = parseLong(v->str);
				if (!b)
				{
					ctx.error(line, "'${" + name + "}' is not an integer; index arithmetic needs one");
					return false;
				}
				base = *b;
			}
			const std::optional<long> n = parseLong(rest);
			if (!n)
			{
				ctx.error(line, "bad index offset in '${" + expr + "}'");
				return false;
			}
			out = std::to_string(base + *n);
			return true;
		}

		// Replaces every ${...} in `in` using `env`. Fast-paths strings with no '${'. On the first
		// unresolved reference it records an Error and returns what it built so far (the caller aborts).
		std::string substitute(Ctx& ctx, const std::string& in, const Env& env, int line)
		{
			if (in.find("${") == std::string::npos)
				return in;
			std::string out;
			out.reserve(in.size());
			std::size_t i = 0;
			while (i < in.size())
			{
				if (in[i] == '$' && i + 1 < in.size() && in[i + 1] == '{')
				{
					const std::size_t close = in.find('}', i + 2);
					if (close == std::string::npos)
					{
						ctx.error(line, "unterminated ${ in '" + in + "'");
						return out;
					}
					std::string val;
					if (!resolveExpr(ctx, trim(in.substr(i + 2, close - (i + 2))), env, line, val))
						return out;
					out += val;
					i = close + 1;
				}
				else
				{
					out += in[i++];
				}
			}
			return out;
		}

		// ── Definition collection ───────────────────────────────────────────────

		// Registers one <pattern-default> from a given source document (`srcText`/`origin`). The name is
		// namespaced by an optional author=, qualified by an optional version=. Conflict policy: a clash
		// within one source is an error; a clash across sources (a library overriding another, or the file
		// overriding a library) is a last-one-wins override with a warning.
		void registerDef(Ctx& ctx, const pugi::xml_node& node, const std::string& srcText, const std::string& origin)
		{
			ctx.sawAny = true;
			const std::string name = node.attribute("name").value();
			const int line = offsetToLine(srcText, node.offset_debug());
			if (name.empty())
			{
				ctx.error(line, "<pattern-default> is missing a name");
				return;
			}
			const std::string author = node.attribute("author").value();
			const std::string fullName = author.empty() ? name : author + "." + name;
			const std::string version = node.attribute("version").value();

			PatternDef def;
			def.src = &srcText;
			def.origin = origin;
			bool haveBody = false;
			for (pugi::xml_node sub : node.children())
			{
				if (sub.type() != pugi::node_element)
					continue;
				const std::string tag = sub.name();
				const int subLine = offsetToLine(srcText, sub.offset_debug());
				if (tag == "param")
				{
					ParamDecl p;
					p.name = sub.attribute("name").value();
					if (p.name.empty())
					{
						ctx.error(subLine, "<param> is missing a name in pattern '" + fullName + "'", fullName);
						return;
					}
					for (const ParamDecl& existing : def.params)
						if (existing.name == p.name)
						{
							ctx.error(subLine, "duplicate <param name='" + p.name + "'> in pattern '" + fullName + "'", fullName);
							return;
						}
					if (pugi::xml_attribute d = sub.attribute("default"))
					{
						p.hasDefault = true;
						p.defVal = d.value();
					}
					def.params.push_back(std::move(p));
				}
				else if (tag == "body")
				{
					if (haveBody)
					{
						ctx.error(subLine, "pattern '" + fullName + "' has more than one <body>", fullName);
						return;
					}
					def.body = sub;
					haveBody = true;
				}
				else
				{
					ctx.error(subLine, "unexpected <" + tag + "> in pattern '" + fullName + "' (only <param>/<body> allowed)", fullName);
					return;
				}
			}
			if (!haveBody)
			{
				ctx.error(line, "pattern '" + fullName + "' has no <body>", fullName);
				return;
			}

			auto& byVersion = ctx.defs[fullName];
			const auto existing = byVersion.find(version);
			if (existing != byVersion.end())
			{
				const std::string vlabel = version.empty() ? "" : (" version '" + version + "'");
				if (existing->second.origin == origin)
				{
					ctx.error(line, "duplicate pattern '" + fullName + "'" + vlabel + " in " + origin, fullName);
					return;
				}
				ctx.warn(line, "pattern '" + fullName + "'" + vlabel + " from " + origin + " overrides the one from " + existing->second.origin, fullName);
			}
			byVersion[version] = std::move(def);
		}

		// Walks a source tree collecting every <pattern-default>. Definitions are not nested inside each
		// other, so we recurse only through ordinary elements.
		void collectDefs(Ctx& ctx, const pugi::xml_node& n, const std::string& srcText, const std::string& origin)
		{
			for (pugi::xml_node child : n.children())
			{
				if (ctx.aborted)
					return;
				if (child.type() != pugi::node_element)
					continue;
				if (std::string(child.name()) == "pattern-default")
					registerDef(ctx, child, srcText, origin);
				else
					collectDefs(ctx, child, srcText, origin);
			}
		}

		// Resolves a <pattern name=... version=...> reference against the registry. Without an explicit
		// version it prefers the unversioned definition, else the sole version, else it is ambiguous.
		const PatternDef* resolveDef(Ctx& ctx, const std::string& name, const std::string& version, int line)
		{
			const auto it = ctx.defs.find(name);
			if (it == ctx.defs.end())
			{
				ctx.error(line, "use of undefined pattern '" + name + "'", name);
				return nullptr;
			}
			const auto& byVersion = it->second;
			if (!version.empty())
			{
				const auto v = byVersion.find(version);
				if (v == byVersion.end())
				{
					ctx.error(line, "pattern '" + name + "' has no version '" + version + "'", name);
					return nullptr;
				}
				return &v->second;
			}
			const auto unversioned = byVersion.find("");
			if (unversioned != byVersion.end())
				return &unversioned->second;
			if (byVersion.size() == 1)
				return &byVersion.begin()->second;
			ctx.error(line, "pattern '" + name + "' has multiple versions; add version=", name);
			return nullptr;
		}

		// ── Expansion ───────────────────────────────────────────────────────────

		void expandChildren(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& src, Env& env, int depth, const std::string& srcText, bool generated);

		// Unrolls a <repeat var count [from]> by binding the loop variable and re-emitting its child
		// template once per index. Nested repeats just recurse, giving 2-D grids.
		void handleRepeat(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& rep, Env& env, int depth, const std::string& srcText, bool generated)
		{
			const int line = offsetToLine(srcText, rep.offset_debug());
			const std::string var = rep.attribute("var").value();
			if (var.empty())
			{
				ctx.error(line, "<repeat> is missing 'var'");
				return;
			}
			pugi::xml_attribute countAttr = rep.attribute("count");
			if (!countAttr)
			{
				ctx.error(line, "<repeat> is missing 'count'");
				return;
			}
			const std::optional<long> count = parseLong(substitute(ctx, countAttr.value(), env, line));
			if (ctx.aborted)
				return;
			if (!count)
			{
				ctx.error(line, "<repeat count> is not an integer");
				return;
			}
			if (*count < 0)
			{
				ctx.error(line, "<repeat count> is negative");
				return;
			}
			if (*count > ctx.limits.maxRepeatCount)
			{
				ctx.error(line, "<repeat count> " + std::to_string(*count) + " exceeds cap " + std::to_string(ctx.limits.maxRepeatCount));
				return;
			}
			long from = 0;
			if (pugi::xml_attribute fromAttr = rep.attribute("from"))
			{
				const std::optional<long> f = parseLong(substitute(ctx, fromAttr.value(), env, line));
				if (ctx.aborted)
					return;
				if (!f)
				{
					ctx.error(line, "<repeat from> is not an integer");
					return;
				}
				from = *f;
			}

			for (long k = 0; k < *count && !ctx.aborted; ++k)
			{
				const long idx = from + k;
				env.push_back({ var, EnvVal{ std::to_string(idx), true, idx } });
				expandChildren(ctx, dest, rep, env, depth, srcText, generated);
				env.pop_back();
			}
		}

		// Replaces a <pattern name=...> use with the parameter-bound body of its definition. Parameter
		// values come from the use's attributes (substituted against the *outer* env, so ${i} from an
		// enclosing repeat can be passed in); the body itself sees ONLY the parameters (hygienic).
		void handlePatternUse(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& use, const Env& outer, int depth, const std::string& srcText)
		{
			const int line = offsetToLine(srcText, use.offset_debug());
			const std::string name = use.attribute("name").value();
			if (name.empty())
			{
				ctx.error(line, "<pattern> is missing a name");
				return;
			}
			const std::string version = use.attribute("version").value();
			const PatternDef* defp = resolveDef(ctx, name, version, line);
			if (!defp)
				return;
			if (depth + 1 > ctx.limits.maxRecursionDepth)
			{
				ctx.error(line, "pattern nesting deeper than " + std::to_string(ctx.limits.maxRecursionDepth) + " (cycle?)", name);
				return;
			}
			const PatternDef& def = *defp;

			// Bind declared params from the use site or their defaults; a missing required param fails.
			Env penv;
			for (const ParamDecl& p : def.params)
			{
				pugi::xml_attribute a = use.attribute(p.name.c_str());
				std::string val;
				if (a)
					val = substitute(ctx, a.value(), outer, line);
				else if (p.hasDefault)
					val = p.defVal;
				else
				{
					ctx.error(line, "pattern '" + name + "' is missing required param '" + p.name + "'", name);
					return;
				}
				if (ctx.aborted)
					return;
				penv.push_back({ p.name, EnvVal{ std::move(val), false, 0 } });
			}
			// Reject stray attributes on the use (catches typo'd param names); name/version are reserved.
			for (pugi::xml_attribute a : use.attributes())
			{
				const std::string an = a.name();
				if (an == "name" || an == "version")
					continue;
				bool declared = false;
				for (const ParamDecl& p : def.params)
					if (p.name == an)
					{
						declared = true;
						break;
					}
				if (!declared)
				{
					ctx.error(line, "pattern '" + name + "' has no param '" + an + "'", name);
					return;
				}
			}

			ctx.sawAny = true;
			ctx.changed = true;
			const pugi::xml_node before = dest.last_child();
			expandChildren(ctx, dest, def.body, penv, depth + 1, *def.src, /*generated=*/true);
			if (ctx.aborted)
				return;

			// Tag the top-level elements this use produced so the serialized output can be sliced into a
			// source-map range. Skip any already tagged: a nested pattern tagged its own output first and
			// innermost attribution wins.
			const int id = static_cast<int>(ctx.patUses.size());
			ctx.patUses.emplace_back(name, line);
			for (pugi::xml_node n = before ? before.next_sibling() : dest.first_child(); n; n = n.next_sibling())
				if (n.type() == pugi::node_element && !n.attribute("_fsmp_pat"))
					n.append_attribute("_fsmp_pat").set_value(std::to_string(id).c_str());
		}

		// Copies `src`'s children into `dest`, transforming as it goes: <pattern-default> dropped,
		// <repeat> unrolled, <pattern> expanded, every other element/text copied with ${...} substituted,
		// and comments/PIs/doctype passed through verbatim.
		void expandChildren(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& src, Env& env, int depth, const std::string& srcText, bool generated)
		{
			for (pugi::xml_node child : src.children())
			{
				if (ctx.aborted)
					return;
				const pugi::xml_node_type t = child.type();
				if (t == pugi::node_element)
				{
					const std::string nm = child.name();
					if (nm == "pattern-default")
						continue;  // definitions are collected, never emitted
					if (nm == "repeat")
					{
						handleRepeat(ctx, dest, child, env, depth, srcText, generated);
						continue;
					}
					if (nm == "pattern")
					{
						handlePatternUse(ctx, dest, child, env, depth, srcText);
						continue;
					}
					const int childLine = offsetToLine(srcText, child.offset_debug());
					if (++ctx.elementCount > ctx.limits.maxExpandedElements)
					{
						ctx.error(childLine, "expanded document exceeds element cap " + std::to_string(ctx.limits.maxExpandedElements));
						return;
					}
					pugi::xml_node ne = dest.append_child(child.name());
					for (pugi::xml_attribute a : child.attributes())
						ne.append_attribute(a.name()).set_value(substitute(ctx, a.value(), env, childLine).c_str());
					// Hand-written (non-generated) elements carry their original line so diagnostics stay
					// correct even though expansion shifts line numbers; generated elements are instead
					// tagged by their <pattern> use in handlePatternUse.
					if (!generated)
						ne.append_attribute("_fsmp_ln").set_value(std::to_string(childLine).c_str());
					expandChildren(ctx, ne, child, env, depth, srcText, generated);
				}
				else if (t == pugi::node_pcdata || t == pugi::node_cdata)
				{
					dest.append_child(t).set_value(substitute(ctx, child.value(), env, offsetToLine(srcText, child.offset_debug())).c_str());
				}
				else if (t == pugi::node_declaration)
				{
					// The output declaration is controlled by the serializer flags; skip the source copy
					// so we never emit it twice.
				}
				else
				{
					dest.append_copy(child);  // comment / PI / doctype: passthrough
				}
			}
		}
	}  // namespace

	const PatternRange* PatternSourceMap::find(std::size_t offset) const
	{
		// Innermost (narrowest) containing range wins, so a pattern nested inside another attributes to
		// the inner use. Ranges are appended outer-before-inner, so a reverse scan hits the inner first.
		for (auto it = ranges.rbegin(); it != ranges.rend(); ++it)
			if (offset >= it->lo && offset < it->hi)
				return &*it;
		return nullptr;
	}

	PatternExpansion expandPatterns(const std::string& raw, const PatternLimits& limits)
	{
		PatternOptions opts;
		opts.limits = limits;
		return expandPatterns(raw, opts);
	}

	PatternExpansion expandPatterns(const std::string& raw, const PatternOptions& options)
	{
		PatternExpansion result;
		result.xml = raw;  // default: hand the original bytes back untouched

		// Short-circuit: a file with no pattern syntax uses no patterns (libraries are then irrelevant), so
		// we never parse or serialize. The overwhelming majority of existing physics files hit this and
		// stay byte-identical at zero cost.
		if (raw.find("<pattern") == std::string::npos)
			return result;

		pugi::xml_document doc;
		const pugi::xml_parse_result pr =
			doc.load_buffer(raw.data(), raw.size(), pugi::parse_full, pugi::encoding_utf8);
		if (!pr)
		{
			result.ok = false;
			result.diags.push_back(
				{ PatternDiagSeverity::Error, std::string("XML parse error: ") + pr.description(), 0, {} });
			return result;
		}

		Ctx ctx{ raw, options.limits };

		// Collect shared-library definitions first (in load order), then the file's own, so that the file
		// -- and later libraries -- override earlier ones.
		if (options.libraries)
		{
			for (const PatternLibrary& lib : *options.libraries)
			{
				if (ctx.aborted)
					break;
				if (lib.xml.find("<pattern-default") == std::string::npos)
					continue;  // nothing to collect from this library; skip the parse
				const std::string origin = lib.origin.empty() ? std::string("<library>") : lib.origin;
				ctx.libDocs.push_back(std::make_unique<pugi::xml_document>());
				pugi::xml_document& ldoc = *ctx.libDocs.back();
				const pugi::xml_parse_result lr =
					ldoc.load_buffer(lib.xml.data(), lib.xml.size(), pugi::parse_full, pugi::encoding_utf8);
				if (!lr)
				{
					ctx.error(0, "pattern library '" + origin + "' parse error: " + lr.description());
					break;
				}
				collectDefs(ctx, ldoc, lib.xml, origin);
			}
		}
		if (!ctx.aborted)
			collectDefs(ctx, doc, raw, "<file>");

		pugi::xml_document out;
		bool hasDecl = false;
		if (!ctx.aborted)
		{
			for (pugi::xml_node c : doc.children())
				if (c.type() == pugi::node_declaration)
					hasDecl = true;
			Env env;
			expandChildren(ctx, out, doc, env, 0, raw, /*generated=*/false);
		}

		result.diags = std::move(ctx.diags);
		if (ctx.aborted)
		{
			result.ok = false;  // result.xml stays == raw
			return result;
		}

		// "<pattern" matched only a comment or attribute, not a real pattern element: nothing was
		// transformed, so hand back the original bytes untouched and keep the byte-identity guarantee.
		if (!ctx.sawAny)
		{
			result.ok = true;
			result.changed = false;
			return result;  // result.xml == raw
		}

		std::ostringstream oss;
		const unsigned flags = hasDecl ? pugi::format_default : (pugi::format_default | pugi::format_no_declaration);
		out.save(oss, "  ", flags);
		const std::string marked = oss.str();

		// Slice the serialized output into source-map ranges and strip the transient _fsmp_pat markers.
		// Each marker tags a pattern-generated top-level element; its subtree span (subtreeEnd) becomes a
		// range, and removing the marker substrings yields the clean output with offsets shifted to match.
		// Two transient markers were planted: _fsmp_pat tags a pattern-generated element (attributed to
		// its pattern + use line) and _fsmp_ln tags a hand-written element (attributed to its original
		// source line, which expansion would otherwise shift). Each marker's subtree span becomes a
		// source-map range; stripping the marker substrings yields the clean output, with range offsets
		// shifted to match.
		struct Marker
		{
			std::size_t lo, hi;
			bool isPattern;
			int value;  // patUses id (pattern) or original line (hand-written)
		};
		std::vector<Marker> markers;
		std::vector<std::pair<std::size_t, std::size_t>> dels;  // marker substrings [start,end) to remove

		const auto scanToken = [&](const std::string& token, bool isPattern) {
			for (std::size_t p = marked.find(token); p != std::string::npos; p = marked.find(token, p + 1))
			{
				const std::size_t vstart = p + token.size();
				const std::size_t vend = marked.find('"', vstart);
				if (vend == std::string::npos)
					break;
				const int value = std::atoi(marked.substr(vstart, vend - vstart).c_str());
				dels.emplace_back(p, vend + 1);
				const std::size_t lt = marked.rfind('<', p);
				const std::size_t lo = (lt == std::string::npos ? p : lt);
				markers.push_back({ lo, subtreeEnd(marked, lo), isPattern, value });
			}
		};
		scanToken(" _fsmp_pat=\"", true);
		scanToken(" _fsmp_ln=\"", false);
		std::sort(dels.begin(), dels.end());
		// Ascending-lo order, so PatternSourceMap::find's reverse scan returns the innermost element.
		std::sort(markers.begin(), markers.end(), [](const Marker& a, const Marker& b) { return a.lo < b.lo; });

		// Clean text = marked text with every marker substring removed.
		std::string clean;
		clean.reserve(marked.size());
		std::size_t cur = 0;
		for (const auto& d : dels)
		{
			clean.append(marked, cur, d.first - cur);
			cur = d.second;
		}
		clean.append(marked, cur, marked.size() - cur);

		// Prefix-sum table so a marked-text offset maps to its clean-text offset in O(log n): subtract the
		// bytes of every marker substring that ends at or before the offset.
		std::vector<std::size_t> delEnd;
		std::vector<std::size_t> removedBefore;  // removedBefore[k] = bytes removed by the first k dels
		removedBefore.push_back(0);
		for (const auto& d : dels)
		{
			delEnd.push_back(d.second);
			removedBefore.push_back(removedBefore.back() + (d.second - d.first));
		}
		const auto mapOffset = [&](std::size_t pos) {
			const std::size_t k = static_cast<std::size_t>(
				std::upper_bound(delEnd.begin(), delEnd.end(), pos) - delEnd.begin());
			return pos - removedBefore[k];
		};

		for (const Marker& m : markers)
		{
			PatternRange r;
			r.lo = mapOffset(m.lo);
			r.hi = mapOffset(m.hi);
			if (m.isPattern)
			{
				if (m.value >= 0 && static_cast<std::size_t>(m.value) < ctx.patUses.size())
				{
					r.patternName = ctx.patUses[m.value].first;
					r.useLine = ctx.patUses[m.value].second;
				}
			}
			else
			{
				r.useLine = m.value;  // hand-written: original source line (patternName stays empty)
			}
			result.sourceMap.ranges.push_back(std::move(r));
		}

		result.xml = std::move(clean);
		result.ok = true;
		result.changed = ctx.changed;
		return result;
	}
}
