<template>
  <div class="flex items-center h-full px-2">
    <TooltipProvider v-if="temp">
      <Tooltip>
        <TooltipTrigger as-child>
          <Badge :class="badgeClass" class="w-12">
            {{ tempShort }} °C
          </Badge>
        </TooltipTrigger>
        <TooltipContent>
          <div :class="badgeClass">{{ temp }} °C</div>
        </TooltipContent>
      </Tooltip>
    </TooltipProvider>
    <Skeleton v-else class="w-12 h-6" />
  </div>
</template>

<script lang="ts" setup>
import { BLE_TEMPERATURE_CHARACTERISTIC, BLE_TEMPERATURE_SERVICE } from '~/lib/bt';

const props = defineProps<{
  gatt: BluetoothRemoteGATTServer | null
}>()

const temp = ref<number | null>()
const tempShort = computed(() => {
  return temp.value?.toFixed(0)
})

watchEffect(async () => {
  if (!props.gatt?.connected) {
    return
  }

  const service = await props.gatt?.getPrimaryService(BLE_TEMPERATURE_SERVICE)

  const char = await service?.getCharacteristic(BLE_TEMPERATURE_CHARACTERISTIC).then(char => char.startNotifications());

  char?.addEventListener('characteristicvaluechanged', e => {
    console.log('Temp value changed')
    if (!e.target) {
      console.error('Invalid characteristic "characteristicvaluechanged" event')
      return
    }

    const value = (e.target as unknown as { value?: DataView }).value;
    const data = value?.getInt16(0, true)
    console.log('got temp', value, data)
    if (!data) {
      temp.value = null
    } else {
      temp.value = data / 100.0
    }
  })
})

// const gatt = toRef(() => props.gatt)
// const service = computedAsync(async () => {
//   if (gatt.value?.connected) {
//     console.log('getting temp service', BLE_TEMPERATURE_SERVICE)
//     const service = await gatt.value.getPrimaryService(BLE_TEMPERATURE_SERVICE)
//     console.log('ble service found', service.getIncludedServices())
//     return service
//   } else {
//     console.log('gatt server disconnected')
//     return null
//   }
// }, null, {
//   lazy: true,
// })

// const char = computedAsync(async () => {
//   if (service.value) {
//     console.log('getting char...', service.value.getIncludedServices())
//     const char = await service.value.getCharacteristic(BLE_TEMPERATURE_CHARACTERISTIC)
//     console.log('ble char found', char)
//     return char
//   } else {
//     console.log('service unavailable...', service.value)
//     return null
//   }
// }, null, {
//   lazy: true,
// })

// const temp = computedAsync(async () => {
//   console.log('getting temp...')
//   if (char.value && char.value.value) {
//     const value = await char.value.readValue();
//     const temp = value.getInt16(0, true)
//     console.log('got temp', value, temp)
//     return temp / 100.0
//   } else {
//     return null
//   }
// }, null, {
//   lazy: true,
// })

const badgeClass = computed(() => {
  if (!temp.value) {
    return ''
  }

  if (temp.value <= 40.0) {
    return 'text-status-ok'
  } else if (temp.value <= 60.0) {
    return 'text-status-warn'
  } else {
    return 'text-status-bad'
  }
})
</script>

<style></style>