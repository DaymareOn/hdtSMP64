#pragma once

#include "Feature.h"

#include <d3d11.h>
#include <winrt/base.h>

// Community Shaders "Strand Hair" feature -- render half of the FSMP wig PoC.
// FSMP (hdtSMP64.dll) simulates guide strands on the physics thread and exports world-space bead
// positions over a C-ABI. This feature resolves that ABI at runtime (GetProcAddress, no link
// dependency), uploads the beads into a StructuredBuffer, and draws them as a line list over the
// opaque scene from a custom PostOpaque() callback.
struct StrandHair : Feature
{
	virtual inline std::string GetName() override { return "Strand Hair"; }
	virtual inline std::string GetShortName() override { return "StrandHair"; }
	virtual inline std::string_view GetCategory() const override { return FeatureCategories::kCharacters; }
	virtual std::pair<std::string, std::vector<std::string>> GetFeatureSummary() override
	{
		return {
			"Renders TressFX-style strand hair simulated by FSMP (hdtSMP64).",
			{ "Guide strands simulated in FSMP, drawn and lit here.", "Proof of concept: player only, line-list strands." }
		};
	}

	virtual void SetupResources() override;
	virtual void PostOpaque() override;  // custom hook added to Feature + Deferred; draws over opaque
	virtual void DrawSettings() override;
	virtual void ClearShaderCache() override;

	// --- FSMP C-ABI bridge (resolved via GetProcAddress on hdtSMP64.dll) ---
	struct FSMPWigInstanceDesc
	{
		std::uint32_t instanceId;
		std::uint32_t strandCount;
		std::uint32_t vertsPerStrand;
		float headWorld[16];
		float colorRoot[3];
		float colorTip[3];
		float roughness;
		float strandRadius;
	};
	using PFN_GetVersion = std::uint32_t (*)();
	using PFN_SetEnabled = void (*)(std::uint32_t);
	using PFN_GetInstanceCount = std::uint32_t (*)();
	using PFN_GetInstance = std::uint32_t (*)(std::uint32_t, FSMPWigInstanceDesc*);
	using PFN_CopyPositions = std::uint32_t (*)(std::uint32_t, float*, std::uint32_t);

	/// Resolve the FSMPWig_* exports once, and (if enabled) opt FSMP's simulation in. Returns
	/// false if hdtSMP64 is absent or too old.
	bool ResolveFSMP();

	struct Settings
	{
		bool enabled = true;
		float radiusScale = 1.0f;
	} settings;

private:
	// One GPU bead: world position + the strand parameter t (0 root .. 1 tip). 16 bytes, matches
	// StructuredBuffer<Bead> in the shader.
	struct Bead
	{
		float pos[3];
		float t;
	};
	// b0 layout for the shader: light parameters only. Camera data comes from the game's own
	// per-frame cbuffer (b12), bound during the pass -- same source as every other CS shader.
	struct StrandCB
	{
		float lightDir[4];
		float lightColor[4];
	};

	void CompileShaders();
	void EnsureBuffers(std::size_t beadCount, std::size_t indexCount);

	PFN_GetVersion m_getVersion = nullptr;
	PFN_SetEnabled m_setEnabled = nullptr;
	PFN_GetInstanceCount m_getInstanceCount = nullptr;
	PFN_GetInstance m_getInstance = nullptr;
	PFN_CopyPositions m_copyPositions = nullptr;
	bool m_resolved = false;

	winrt::com_ptr<ID3D11VertexShader> m_vs;
	winrt::com_ptr<ID3D11PixelShader> m_ps;
	winrt::com_ptr<ID3D11Buffer> m_beadBuffer;   // dynamic StructuredBuffer<Bead>
	winrt::com_ptr<ID3D11ShaderResourceView> m_beadSRV;
	winrt::com_ptr<ID3D11Buffer> m_indexBuffer;  // dynamic line-list indices
	winrt::com_ptr<ID3D11Buffer> m_strandCB;
	winrt::com_ptr<ID3D11BlendState> m_blendState;
	winrt::com_ptr<ID3D11DepthStencilState> m_depthState;
	winrt::com_ptr<ID3D11RasterizerState> m_rasterState;
	std::size_t m_beadCapacity = 0;
	std::size_t m_indexCapacity = 0;

	// CPU staging reused each frame.
	std::vector<float> m_posScratch;
	std::vector<Bead> m_beadScratch;
	std::vector<std::uint32_t> m_indexScratch;
};
