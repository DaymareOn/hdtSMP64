#include "StrandConfig.h"

#include "NetImmerseUtils.h"
#include "XmlReader.h"
#include "hdtStringUtils.h"

#include <algorithm>
#include <cctype>

namespace hdt
{
	// Reduce a config-supplied groom filename to a safe bare filename, or "" if it is unsafe. This
	// is a trust boundary: the value gets appended to a fixed grooms/ path, so anything that could
	// escape that folder (path separators, "..", drive/absolute markers) is rejected outright.
	static std::string sanitizeGroomFile(const std::string& raw)
	{
		const std::string s = TrimAsciiWhitespace(raw);
		if (s.empty() || s.size() > 128)
			return {};
		if (s.find("..") != std::string::npos)
			return {};
		for (const char c : s) {
			const bool ok = std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' || c == '-' || c == ' ';
			if (!ok)  // rejects '/', '\\', ':' and any other separator/control char
				return {};
		}
		return s;
	}

	bool loadStrandConfig(const std::string& path, StrandConfig& out)
	{
		std::string data = readAllFile2(path.c_str());
		if (data.empty())
			return false;  // no file -> keep defaults

		// The reader's read* helpers throw std::string on a malformed number/bool. Catch here so a
		// bad config file fails closed (keeps whatever was parsed so far + defaults) instead of
		// throwing across the trust boundary into the caller.
		try {
			XMLReader reader(reinterpret_cast<BYTE*>(data.data()), data.size());
			while (reader.Inspect()) {
				if (reader.GetInspected() != XMLReader::Inspected::StartTag)
					continue;
				const auto name = reader.GetName();
				if (name == "strands")
					out.strandCount = reader.readInt();
				else if (name == "verts-per-strand")
					out.vertsPerStrand = reader.readInt();
				else if (name == "length")
					out.length = reader.readFloat();
				else if (name == "length-variation")
					out.lengthVariation = reader.readFloat();
				else if (name == "stiffness")
					out.stiffness = reader.readFloat();
				else if (name == "damping")
					out.damping = reader.readFloat();
				else if (name == "color-root") {
					const auto v = reader.readVector3();  // x/y/z attributes = linear RGB
					out.colorRoot[0] = static_cast<float>(v.x());
					out.colorRoot[1] = static_cast<float>(v.y());
					out.colorRoot[2] = static_cast<float>(v.z());
				} else if (name == "color-tip") {
					const auto v = reader.readVector3();
					out.colorTip[0] = static_cast<float>(v.x());
					out.colorTip[1] = static_cast<float>(v.y());
					out.colorTip[2] = static_cast<float>(v.z());
				} else if (name == "width")
					out.width = reader.readFloat();
				else if (name == "groom")
					out.groomFile = sanitizeGroomFile(reader.readText());
				else if (name == "groom-scale")
					out.groomScale = reader.readFloat();
				else if (name == "wig-armor-only")
					out.attachToWigArmorOnly = reader.readBool();
				// unknown tags: ignored (readText/readFloat not called, so the reader steps past them)
			}
		} catch (...) {
			// Malformed value: stop parsing; the clamp below keeps everything in-range.
		}

		// Clamp every value read across the trust boundary to a solver-safe range (fail closed).
		out.strandCount = std::clamp(out.strandCount, 1, 4096);
		out.vertsPerStrand = std::clamp(out.vertsPerStrand, 2, 64);
		out.length = std::clamp(out.length, 1.0f, 500.0f);
		out.lengthVariation = std::clamp(out.lengthVariation, 0.0f, 1.0f);
		out.stiffness = std::clamp(out.stiffness, 0.0f, 1.0f);
		out.damping = std::clamp(out.damping, 0.0f, 1.0f);
		for (float& c : out.colorRoot)
			c = std::clamp(c, 0.0f, 1.0f);
		for (float& c : out.colorTip)
			c = std::clamp(c, 0.0f, 1.0f);
		out.width = std::clamp(out.width, 0.01f, 5.0f);
		out.groomScale = std::clamp(out.groomScale, 0.001f, 1000.0f);
		return true;
	}
}
