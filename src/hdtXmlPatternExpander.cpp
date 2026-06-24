#include "hdtXmlPatternExpander.h"

#include <pugixml.hpp>

#include <cctype>
#include <map>
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
			pugi::xml_node body;  // points into the original parsed document
		};

		// Shared mutable state for one expansion run. `aborted` latches on the first Error so the
		// recursion unwinds promptly and the caller discards the half-built tree (fail closed).
		struct Ctx
		{
			const std::string& raw;
			const PatternLimits& limits;
			std::map<std::string, PatternDef> defs;
			std::vector<PatternDiag> diags;
			std::size_t elementCount = 0;
			bool aborted = false;
			bool changed = false;

			void error(int line, std::string msg, std::string pat = {})
			{
				diags.push_back({ PatternDiagSeverity::Error, std::move(msg), line, std::move(pat) });
				aborted = true;
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

		int lineOf(const Ctx& ctx, const pugi::xml_node& n)
		{
			return offsetToLine(ctx.raw, n.offset_debug());
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

		void registerDef(Ctx& ctx, const pugi::xml_node& node)
		{
			const std::string name = node.attribute("name").value();
			const int line = lineOf(ctx, node);
			if (name.empty())
			{
				ctx.error(line, "<pattern-default> is missing a name");
				return;
			}
			if (ctx.defs.count(name))
			{
				ctx.error(line, "duplicate <pattern-default name='" + name + "'>", name);
				return;
			}

			PatternDef def;
			bool haveBody = false;
			for (pugi::xml_node sub : node.children())
			{
				if (sub.type() != pugi::node_element)
					continue;
				const std::string tag = sub.name();
				if (tag == "param")
				{
					ParamDecl p;
					p.name = sub.attribute("name").value();
					if (p.name.empty())
					{
						ctx.error(lineOf(ctx, sub), "<param> is missing a name in pattern '" + name + "'", name);
						return;
					}
					for (const ParamDecl& existing : def.params)
						if (existing.name == p.name)
						{
							ctx.error(lineOf(ctx, sub), "duplicate <param name='" + p.name + "'> in pattern '" + name + "'", name);
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
						ctx.error(lineOf(ctx, sub), "pattern '" + name + "' has more than one <body>", name);
						return;
					}
					def.body = sub;
					haveBody = true;
				}
				else
				{
					ctx.error(lineOf(ctx, sub), "unexpected <" + tag + "> in pattern '" + name + "> (only <param>/<body> allowed)", name);
					return;
				}
			}
			if (!haveBody)
			{
				ctx.error(line, "pattern '" + name + "' has no <body>", name);
				return;
			}
			ctx.defs.emplace(name, std::move(def));
		}

		// Walks the original tree collecting every <pattern-default>. Definitions are not nested inside
		// each other, so we recurse only through ordinary elements.
		void collectDefs(Ctx& ctx, const pugi::xml_node& n)
		{
			for (pugi::xml_node child : n.children())
			{
				if (ctx.aborted)
					return;
				if (child.type() != pugi::node_element)
					continue;
				if (std::string(child.name()) == "pattern-default")
					registerDef(ctx, child);
				else
					collectDefs(ctx, child);
			}
		}

		// ── Expansion ───────────────────────────────────────────────────────────

		void expandChildren(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& src, Env& env, int depth);

		// Unrolls a <repeat var count [from]> by binding the loop variable and re-emitting its child
		// template once per index. Nested repeats just recurse, giving 2-D grids.
		void handleRepeat(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& rep, Env& env, int depth)
		{
			const int line = lineOf(ctx, rep);
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
				expandChildren(ctx, dest, rep, env, depth);
				env.pop_back();
			}
		}

		// Replaces a <pattern name=...> use with the parameter-bound body of its definition. Parameter
		// values come from the use's attributes (substituted against the *outer* env, so ${i} from an
		// enclosing repeat can be passed in); the body itself sees ONLY the parameters (hygienic).
		void handlePatternUse(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& use, const Env& outer, int depth)
		{
			const int line = lineOf(ctx, use);
			const std::string name = use.attribute("name").value();
			if (name.empty())
			{
				ctx.error(line, "<pattern> is missing a name");
				return;
			}
			const auto it = ctx.defs.find(name);
			if (it == ctx.defs.end())
			{
				ctx.error(line, "use of undefined pattern '" + name + "'", name);
				return;
			}
			if (depth + 1 > ctx.limits.maxRecursionDepth)
			{
				ctx.error(line, "pattern nesting deeper than " + std::to_string(ctx.limits.maxRecursionDepth) + " (cycle?)", name);
				return;
			}
			const PatternDef& def = it->second;

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
			// Reject stray attributes on the use site (catches typo'd param names instead of ignoring them).
			for (pugi::xml_attribute a : use.attributes())
			{
				const std::string an = a.name();
				if (an == "name")
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

			ctx.changed = true;
			expandChildren(ctx, dest, def.body, penv, depth + 1);
		}

		// Copies `src`'s children into `dest`, transforming as it goes: <pattern-default> dropped,
		// <repeat> unrolled, <pattern> expanded, every other element/text copied with ${...} substituted,
		// and comments/PIs/doctype passed through verbatim.
		void expandChildren(Ctx& ctx, pugi::xml_node dest, const pugi::xml_node& src, Env& env, int depth)
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
						handleRepeat(ctx, dest, child, env, depth);
						continue;
					}
					if (nm == "pattern")
					{
						handlePatternUse(ctx, dest, child, env, depth);
						continue;
					}
					if (++ctx.elementCount > ctx.limits.maxExpandedElements)
					{
						ctx.error(lineOf(ctx, child), "expanded document exceeds element cap " + std::to_string(ctx.limits.maxExpandedElements));
						return;
					}
					pugi::xml_node ne = dest.append_child(child.name());
					for (pugi::xml_attribute a : child.attributes())
						ne.append_attribute(a.name()).set_value(substitute(ctx, a.value(), env, lineOf(ctx, child)).c_str());
					expandChildren(ctx, ne, child, env, depth);
				}
				else if (t == pugi::node_pcdata || t == pugi::node_cdata)
				{
					dest.append_child(t).set_value(substitute(ctx, child.value(), env, lineOf(ctx, child)).c_str());
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
		PatternExpansion result;
		result.xml = raw;  // default: hand the original bytes back untouched

		// Short-circuit: no pattern syntax at all -> never parse or serialize. The overwhelming majority
		// of existing physics files hit this and stay byte-identical at zero cost.
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

		Ctx ctx{ raw, limits };
		collectDefs(ctx, doc);

		pugi::xml_document out;
		bool hasDecl = false;
		if (!ctx.aborted)
		{
			for (pugi::xml_node c : doc.children())
				if (c.type() == pugi::node_declaration)
					hasDecl = true;
			Env env;
			expandChildren(ctx, out, doc, env, 0);
		}

		result.diags = std::move(ctx.diags);
		if (ctx.aborted)
		{
			result.ok = false;  // result.xml stays == raw
			return result;
		}

		std::ostringstream oss;
		const unsigned flags = hasDecl ? pugi::format_default : (pugi::format_default | pugi::format_no_declaration);
		out.save(oss, "  ", flags);
		result.xml = oss.str();
		result.ok = true;
		result.changed = ctx.changed;
		return result;
	}
}
