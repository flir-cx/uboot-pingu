#include <ec201_splash.h>

static struct splash_location splash_locations[] = {
	{
		.name = "mmc_fs",
		.storage = SPLASH_STORAGE_MMC,
		.flags = SPLASH_STORAGE_FS,
		.devpart = "0:2",
	}
};

#ifdef CONFIG_VIDEO_MXS
int splash_screen_prepare(void)
{
	//choose which partition to load bootlogo from
	char * system_active = env_get("system_active");
	if(system_active)
	{
		switch(system_active[6]) //system_active=system1 or system2
		{
		case '1':
			splash_locations[0].devpart[2]='2'; //use mmc partition 2
			break;
		case '2':
			splash_locations[0].devpart[2]='3'; //use mmc partition 3
			break;
		default:
			printf("splash_screen_prepare: invalid system_active environment: %s \n",system_active);
			break;
		}
	}
	return splash_source_load(splash_locations,
				ARRAY_SIZE(splash_locations));
}

void splash_screen_update(void)
{
	char *s;
	ulong addr;

	s = env_get("splashimage");
	if(s == NULL){
		printf("Failed to update splash: environment variable splashimage is null\n");
		return;
	}

	if(splash_source_load(splash_locations, ARRAY_SIZE(splash_locations))){
		printf("Failed to load splash\n");
		return;
	}

	addr = simple_strtoul(s, NULL, 16);
	if(video_display_bitmap(addr, 0, 0))
		printf("Failed to update splash: failed to display bitmap\n");
}
#endif
