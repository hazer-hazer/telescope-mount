<template>
  <div class="flex">
    <div class="flex flex-1/2 flex-col">
      <div class="text-2xl">
        ???
      </div>

      <div class="flex flex-col">
        <div class="flex flex-row">
          <div class="text-secondary">
            <span>Ra/Dec</span>
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger as-child>
                  <Icon name="lucide-circle-question-mark" />
                </TooltipTrigger>
                <TooltipContent>
                  <p>Right Ascension & Declination determines global sky coordinates</p>
                </TooltipContent>
              </Tooltip>
            </TooltipProvider>
          </div>
          <div class="text-primary" />
        </div>

        <div class="flex flex-row">
          <div class="text-secondary">Chip temp</div>
          <div class="text-primary">
            <BoardTemp :gatt="gatt" />
          </div>
        </div>
      </div>
    </div>

    <div class="flex flex-col items-center space-y-2">
      <Switch v-model="manualMode" class="flex items-center">
        <template #thumb>
          <Icon :name="manualMode ? 'lucide-hand' : 'lucide-telescope'" :class="manualModeIconClass"
            class="relative bottom-1/5 mx-auto" size="14" />
        </template>
      </Switch>
      <Label>
        {{ manualModeText }}
      </Label>
    </div>

  </div>
</template>

<script lang="ts" setup>

const props = defineProps<{
  gatt: BluetoothRemoteGATTServer | null
}>()
// const gatt = toRef(() => props.gatt)

const colorMode = useColorMode()

// TODO: Remote state
const manualMode = ref(false)
const manualModeText = computed(() => {
  if (manualMode.value) {
    return 'Manual mode'
  } else {
    return 'Track mode'
  }
})

const manualModeIconClass = computed(() => {
  if (colorMode.value === 'dark' && !manualMode.value) {
    return 'text-background'
  } else {
    return ''
  }
})

const coords = computed(() => {
  // RA/Dec coords in seconds and arcsecs 
  const fakeCoords = [13500, 1331.9]

  return fakeCoords
})

// const raDec = 

</script>

<style></style>
