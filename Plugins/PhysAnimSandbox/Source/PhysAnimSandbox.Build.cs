namespace UnrealBuildTool.Rules
{
	public class PhysAnimSandbox : ModuleRules
	{
		public PhysAnimSandbox(ReadOnlyTargetRules Target) : base(Target)
		{
			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
					"Engine"
				}
				);
		}
	}
}

