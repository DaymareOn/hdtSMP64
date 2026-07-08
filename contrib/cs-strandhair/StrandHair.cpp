#include "Features/StrandHair.h"

#include "Globals.h"
#include "State.h"
#include "Utils/D3D.h"

bool StrandHair::ResolveFSMP()
{
	if (m_resolved)
		return true;
	HMODULE h = GetModuleHandleA("hdtSMP64.dll");
	if (!h)
		h = GetModuleHandleA("hdtsmp64.dll");
	if (!h)
		return false;

	m_getVersion = reinterpret_cast<PFN_GetVersion>(GetProcAddress(h, "FSMPWig_GetVersion"));
	m_setEnabled = reinterpret_cast<PFN_SetEnabled>(GetProcAddress(h, "FSMPWig_SetEnabled"));
	m_getInstanceCount = reinterpret_cast<PFN_GetInstanceCount>(GetProcAddress(h, "FSMPWig_GetInstanceCount"));
	m_getInstance = reinterpret_cast<PFN_GetInstance>(GetProcAddress(h, "FSMPWig_GetInstance"));
	m_copyPositions = reinterpret_cast<PFN_CopyPositions>(GetProcAddress(h, "FSMPWig_CopyPositions"));

	m_resolved = m_getVersion && m_setEnabled && m_getInstanceCount && m_getInstance && m_copyPositions;
	if (m_resolved && m_getVersion() >= 1)
		m_setEnabled(settings.enabled ? 1 : 0);  // gate FSMP's simulation on this feature
	return m_resolved;
}

void StrandHair::CompileShaders()
{
	// Util::CompileShader creates the shader and discards the blob; we don't need the blob because
	// the VS reads beads from a StructuredBuffer by SV_VertexID (no input layout required).
	if (auto p = reinterpret_cast<ID3D11VertexShader*>(
			Util::CompileShader(L"Data\\Shaders\\StrandHair\\StrandHair.hlsl", {}, "vs_5_0", "VSMain")))
		m_vs.attach(p);
	if (auto p = reinterpret_cast<ID3D11PixelShader*>(
			Util::CompileShader(L"Data\\Shaders\\StrandHair\\StrandHair.hlsl", {}, "ps_5_0", "PSMain")))
		m_ps.attach(p);
}

void StrandHair::SetupResources()
{
	CompileShaders();

	auto device = globals::d3d::device;
	if (!device)
		return;

	{  // light-parameter constant buffer (b0)
		D3D11_BUFFER_DESC cd{};
		cd.ByteWidth = sizeof(StrandCB);
		cd.Usage = D3D11_USAGE_DYNAMIC;
		cd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		cd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		device->CreateBuffer(&cd, nullptr, m_strandCB.put());
	}
	{  // straight alpha blend so overlapping strands composite
		D3D11_BLEND_DESC bd{};
		auto& rt = bd.RenderTarget[0];
		rt.BlendEnable = TRUE;
		rt.SrcBlend = D3D11_BLEND_SRC_ALPHA;
		rt.DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
		rt.BlendOp = D3D11_BLEND_OP_ADD;
		rt.SrcBlendAlpha = D3D11_BLEND_ONE;
		rt.DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
		rt.BlendOpAlpha = D3D11_BLEND_OP_ADD;
		rt.RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
		device->CreateBlendState(&bd, m_blendState.put());
	}
	{  // Depth test against the scene (LESS_EQUAL) so heads/walls occlude hair; no depth write,
	   // since alpha-blended ribbons must not punch holes into later transparents.
		D3D11_DEPTH_STENCIL_DESC dd{};
		dd.DepthEnable = TRUE;
		dd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
		dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
		device->CreateDepthStencilState(&dd, m_depthState.put());
	}
	{  // solid, no cull (lines aren't culled anyway)
		D3D11_RASTERIZER_DESC rd{};
		rd.FillMode = D3D11_FILL_SOLID;
		rd.CullMode = D3D11_CULL_NONE;
		rd.DepthClipEnable = TRUE;
		device->CreateRasterizerState(&rd, m_rasterState.put());
	}
}

void StrandHair::EnsureBuffers(std::size_t beadCount, std::size_t indexCount)
{
	auto device = globals::d3d::device;
	if (!device)
		return;

	if (beadCount > m_beadCapacity || !m_beadBuffer) {
		m_beadCapacity = std::max<std::size_t>(beadCount, 4096);
		m_beadBuffer = nullptr;
		m_beadSRV = nullptr;
		D3D11_BUFFER_DESC bd{};
		bd.ByteWidth = static_cast<UINT>(m_beadCapacity * sizeof(Bead));
		bd.Usage = D3D11_USAGE_DYNAMIC;
		bd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
		bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		bd.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
		bd.StructureByteStride = sizeof(Bead);
		device->CreateBuffer(&bd, nullptr, m_beadBuffer.put());
		if (m_beadBuffer) {
			D3D11_SHADER_RESOURCE_VIEW_DESC sd{};
			sd.Format = DXGI_FORMAT_UNKNOWN;
			sd.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
			sd.Buffer.FirstElement = 0;
			sd.Buffer.NumElements = static_cast<UINT>(m_beadCapacity);
			device->CreateShaderResourceView(m_beadBuffer.get(), &sd, m_beadSRV.put());
		}
	}
	if (indexCount > m_indexCapacity || !m_indexBuffer) {
		m_indexCapacity = std::max<std::size_t>(indexCount, 8192);
		m_indexBuffer = nullptr;
		D3D11_BUFFER_DESC id{};
		id.ByteWidth = static_cast<UINT>(m_indexCapacity * sizeof(std::uint32_t));
		id.Usage = D3D11_USAGE_DYNAMIC;
		id.BindFlags = D3D11_BIND_INDEX_BUFFER;
		id.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		device->CreateBuffer(&id, nullptr, m_indexBuffer.put());
	}
}

void StrandHair::PostOpaque()
{
	if (!settings.enabled || !ResolveFSMP())
		return;
	// Recompile after a shader-cache clear (ClearShaderCache nulls the shaders).
	if (!m_vs || !m_ps)
		CompileShaders();
	if (!m_vs || !m_ps || !m_strandCB)
		return;

	auto context = globals::d3d::context;
	auto renderer = globals::game::renderer;
	if (!context || !renderer)
		return;

	const std::uint32_t count = m_getInstanceCount();
	if (count == 0)
		return;

	// Build CPU staging from every published wig: one Bead per position, one line per segment.
	m_beadScratch.clear();
	m_indexScratch.clear();
	for (std::uint32_t i = 0; i < count; ++i) {
		FSMPWigInstanceDesc d{};
		if (!m_getInstance(i, &d) || d.strandCount == 0 || d.vertsPerStrand < 2)
			continue;

		const std::uint32_t beads = d.strandCount * d.vertsPerStrand;
		m_posScratch.resize(static_cast<std::size_t>(beads) * 3);
		if (m_copyPositions(i, m_posScratch.data(), static_cast<std::uint32_t>(m_posScratch.size())) < beads * 3)
			continue;

		const std::uint32_t vbase = static_cast<std::uint32_t>(m_beadScratch.size());
		for (std::uint32_t s = 0; s < d.strandCount; ++s) {
			for (std::uint32_t v = 0; v < d.vertsPerStrand; ++v) {
				const std::size_t b = (static_cast<std::size_t>(s) * d.vertsPerStrand + v) * 3;
				Bead bead{};
				bead.pos[0] = m_posScratch[b + 0];
				bead.pos[1] = m_posScratch[b + 1];
				bead.pos[2] = m_posScratch[b + 2];
				bead.t = static_cast<float>(v) / static_cast<float>(d.vertsPerStrand - 1);
				m_beadScratch.push_back(bead);
			}
			// Each segment is a camera-facing quad over the doubled vertex ids the VS expands
			// (bead i -> vertices 2i / 2i+1 for the ribbon's two sides).
			for (std::uint32_t v = 0; v + 1 < d.vertsPerStrand; ++v) {
				const std::uint32_t b0 = (vbase + s * d.vertsPerStrand + v) * 2;
				const std::uint32_t b1 = b0 + 2;
				m_indexScratch.push_back(b0);
				m_indexScratch.push_back(b0 + 1);
				m_indexScratch.push_back(b1);
				m_indexScratch.push_back(b0 + 1);
				m_indexScratch.push_back(b1 + 1);
				m_indexScratch.push_back(b1);
			}
		}

	}
	if (m_beadScratch.empty() || m_indexScratch.empty())
		return;

	EnsureBuffers(m_beadScratch.size(), m_indexScratch.size());
	if (!m_beadBuffer || !m_beadSRV || !m_indexBuffer)
		return;

	D3D11_MAPPED_SUBRESOURCE mapped{};
	if (SUCCEEDED(context->Map(m_beadBuffer.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
		std::memcpy(mapped.pData, m_beadScratch.data(), m_beadScratch.size() * sizeof(Bead));
		context->Unmap(m_beadBuffer.get(), 0);
	}
	if (SUCCEEDED(context->Map(m_indexBuffer.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
		std::memcpy(mapped.pData, m_indexScratch.data(), m_indexScratch.size() * sizeof(std::uint32_t));
		context->Unmap(m_indexBuffer.get(), 0);
	}
	if (SUCCEEDED(context->Map(m_strandCB.get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &mapped))) {
		auto* cb = static_cast<StrandCB*>(mapped.pData);
		const float pr[4] = { 0.4f * settings.radiusScale, 0.0f, 0.0f, 0.0f };  // ribbon half-width
		std::memcpy(cb->params, pr, sizeof(pr));
		context->Unmap(m_strandCB.get(), 0);
	}

	// Save current RTV/DSV so we hand the pipeline back exactly as we found it.
	ID3D11RenderTargetView* savedRTV = nullptr;
	ID3D11DepthStencilView* savedDSV = nullptr;
	context->OMGetRenderTargets(1, &savedRTV, &savedDSV);

	auto& main = renderer->GetRuntimeData().renderTargets[RE::RENDER_TARGETS::kMAIN];
	ID3D11RenderTargetView* rtv = main.RTV;
	ID3D11DepthStencilView* dsv = renderer->GetDepthStencilData().depthStencils[RE::RENDER_TARGETS_DEPTHSTENCIL::kMAIN].views[0];
	context->OMSetRenderTargets(1, &rtv, dsv);  // scene depth: heads/walls occlude the ribbons

	const float blendFactor[4] = { 0, 0, 0, 0 };
	context->OMSetBlendState(m_blendState.get(), blendFactor, 0xFFFFFFFF);
	context->OMSetDepthStencilState(m_depthState.get(), 0);
	context->RSSetState(m_rasterState.get());
	context->IASetInputLayout(nullptr);
	context->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	context->IASetIndexBuffer(m_indexBuffer.get(), DXGI_FORMAT_R32_UINT, 0);
	ID3D11Buffer* nullVB = nullptr;
	UINT zero = 0;
	context->IASetVertexBuffers(0, 1, &nullVB, &zero, &zero);
	context->VSSetShader(m_vs.get(), nullptr, 0);
	context->PSSetShader(m_ps.get(), nullptr, 0);
	ID3D11ShaderResourceView* srv = m_beadSRV.get();
	context->VSSetShaderResources(0, 1, &srv);
	ID3D11Buffer* cb = m_strandCB.get();
	context->VSSetConstantBuffers(0, 1, &cb);
	context->PSSetConstantBuffers(0, 1, &cb);
	// The game's live per-frame cbuffer (b12): the exact camera matrices the opaque scene was
	// just rendered with -- same binding Deferred uses for compute.
	ID3D11Buffer* perFrame = *globals::game::perFrame.get();
	context->VSSetConstantBuffers(12, 1, &perFrame);
	// CS's SharedData (b5): scene sun direction/colour + directional ambient, updated pre-opaque.
	ID3D11Buffer* sharedData = globals::state->sharedDataCB->CB();
	context->PSSetConstantBuffers(5, 1, &sharedData);

	context->DrawIndexed(static_cast<UINT>(m_indexScratch.size()), 0, 0);

	// Unbind our SRV and restore render targets; flag the engine to rebind its state next draw.
	ID3D11ShaderResourceView* nullSRV = nullptr;
	context->VSSetShaderResources(0, 1, &nullSRV);
	context->OMSetRenderTargets(1, &savedRTV, savedDSV);
	if (savedRTV)
		savedRTV->Release();
	if (savedDSV)
		savedDSV->Release();
	globals::game::stateUpdateFlags->set(RE::BSGraphics::ShaderFlags::DIRTY_RENDERTARGET);
}

void StrandHair::DrawSettings()
{
	ImGui::Checkbox("Enabled", &settings.enabled);
	if (m_setEnabled)
		m_setEnabled(settings.enabled ? 1 : 0);
	ImGui::SliderFloat("Strand radius scale", &settings.radiusScale, 0.25f, 4.0f);
	ImGui::Text("FSMP: %s", m_resolved ? "connected" : "not found");
	if (m_resolved && m_getInstanceCount)
		ImGui::Text("Wigs this frame: %u", m_getInstanceCount());
}

void StrandHair::ClearShaderCache()
{
	m_vs = nullptr;
	m_ps = nullptr;
}
