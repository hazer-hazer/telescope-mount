<template>
  <div class="flex flex-col h-screen">
    <main v-if="isSupported" class="flex mb-auto">
      <Button variant="outline" class="m-auto my-2" @click="connect">
        <Icon name="lucide-bluetooth" class="inline mr-2" />
        Connect
      </Button>
    </main>
    <main v-else class="flex mb-auto">
      <p class="m-auto my-2 text-red-500">Bluetooth is not supported on this device.</p>
    </main>
    <footer class="flex sticky bottom-0 h-10 justify-center items-center text-white border-t-1 text-sm">
      <ConnectionStatus :kind="status" :error="error" />
    </footer>
  </div>
</template>

<script setup lang="ts">
import { BLE_SERVICES } from '~/lib/bt';
import { BluetoothDeviceMock, BluetoothMock, ServiceMock } from '~/lib/bt-mock';
import { ConnectionState } from '~/types/connection';

const mockDevice = () => {
  const device = new BluetoothDeviceMock('keklol', [
    new ServiceMock(0x1809, )
  ])
  navigator.bluetooth = new BluetoothMock()
}

const { isSupported, requestDevice, isConnected, error, device } = useBluetooth({
  acceptAllDevices: true,
  filters: [{
    name: 'telemount',
  }],
  optionalServices: BLE_SERVICES,
})

const router = useRouter()
const status = ref(ConnectionState.Ready)
const store = useMyBtStore()

const connect = async () => {
  status.value = ConnectionState.Connecting
  await requestDevice()
}

watchEffect(() => {
  if (isConnected.value) {
    status.value = ConnectionState.Connected
  } else if (error) {
    status.value = ConnectionState.Error
  }

  if (isConnected.value && device.value?.name) {
    store.connected(device.value.id, device.value.name)

    router.push('/control')
  }
})
</script>

<style scoped lang="scss"></style>
