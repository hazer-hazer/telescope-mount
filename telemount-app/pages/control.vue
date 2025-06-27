<template>
  <div class="flex flex-col h-screen">
    <main v-if="status === ConnectionState.Connected" class="flex flex-row mb-auto justify-center p-2">
      <ControlsRemote class="flex mb-auto" :gatt="gatt" />
    </main>
    <main v-else class="mb-auto justify-center items-center">
      <div class="my-2 flex mx-auto w-fit gap-2 items-center">
        <Skeleton class="w-10 h-10 mx-auto rounded-4xl" />
        <Skeleton class="w-64 h-10 mx-auto rounded-4xl" />
        <Skeleton class="w-10 h-10 mx-auto rounded-4xl" />
      </div>
      <div class="flex mx-auto w-fit gap-2 items-center">
        <Skeleton class="w-10 h-64 mx-auto rounded-4xl" />
        <Skeleton class="w-64 h-64 mx-auto rounded-4xl" />
        <Skeleton class="w-10 h-64 mx-auto rounded-4xl" />
      </div>
      <div class="my-2 flex mx-auto w-fit gap-2 items-center">
        <Skeleton class="w-10 h-10 mx-auto rounded-4xl" />
        <Skeleton class="w-64 h-10 mx-auto rounded-4xl" />
        <Skeleton class="w-10 h-10 mx-auto rounded-4xl" />
      </div>
    </main>
    <footer class="flex w-full sticky gap-5 bottom-0 h-10 justify-center items-center text-white border-t-1">
      <Button v-if="gatt?.connected" class="absolute left-0 rounded-r-2xl rounded-l-none h-full" variant="destructive"
        @click="disconnect">Disconnect</Button>
      <ConnectionStatus class="mx-auto" :kind="status" :error="error" />
    </footer>
  </div>
</template>

<script lang="ts" setup>
import { useMyBtStore } from '#imports'
import { ConnectionState } from '~/types/connection'

const store = useMyBtStore()

const status = ref(ConnectionState.Connecting)
const error = ref<unknown | null>(null)

watchEffect(() => {
  if (!store.id) {
    useRouter().push('/connect')
  }
})

const device = await navigator.bluetooth.getDevices().then(devices => {
  return devices.find(d => d.id === store.id)
})
// GATT server to be passed to components
const gatt = ref<BluetoothRemoteGATTServer | null>(null)

const disconnect = async () => {
  gatt.value?.disconnect()
  useMyBtStore().forget()
}

console.log('device', device)
if (!device || !device.gatt) {
  useRouter().push('/connect')
}

device?.addEventListener('advertisementreceived', async _ => {
  try {
    status.value = ConnectionState.Connecting
    if (!gatt?.value?.connected) {
      await device?.gatt?.connect().then(async server => {
        console.log('connected to GATT server', server)

        const services = await server.getPrimaryServices()
        const servicesChars = await Promise.all(services.map(async service => {
          const chars = await service.getCharacteristics().then(chars => chars.map(char => char.uuid));
          return [service.uuid, chars]
        }));

        console.group('BLE services')
        servicesChars.forEach(([service, chars]) => {
          console.group(`Service ${service}`)
          console.table(chars)
          console.groupEnd()
        })
        console.groupEnd()

        console.log('set gatt')
        status.value = ConnectionState.Connected
        gatt.value = server
      }).catch(error => {
        console.error('Failed to connect to GATT server:', error)
      })
    }
  } catch (err) {
    error.value = err;
    gatt.value = null;

    useRouter().push('/connect')
  }
})

device?.addEventListener('gattserverdisconnected', async _ => {
  status.value = ConnectionState.Disconnected
  gatt.value = null
})

await device?.watchAdvertisements()


// const services = await device.gatt.connect().then(server => {
//   return server.getPrimaryServices()
// })
</script>


<style></style>
