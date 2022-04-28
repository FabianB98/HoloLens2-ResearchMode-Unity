#pragma once
#include <winrt/Windows.Foundation.Numerics.h>

namespace winrt::HL2UnityPlugin::implementation
{
	class PixelDirection
	{
	public:
		int u;
		int v;
		Windows::Foundation::Numerics::float3 direction;

		PixelDirection(int u0, int v0, Windows::Foundation::Numerics::float3 direction0)
		{
			u = u0;
			v = v0;
			direction = direction0;
		}
	};
}