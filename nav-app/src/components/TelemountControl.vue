<template>
  <div>
    <Menubar class="menubar">
      <MenubarMenu>
        <MenubarTrigger>File</MenubarTrigger>
        <MenubarContent>
          <MenubarItem>
            New Tab <MenubarShortcut>⌘T</MenubarShortcut>
          </MenubarItem>
          <MenubarItem>New Window</MenubarItem>
          <MenubarSeparator />
          <MenubarItem>Share</MenubarItem>
          <MenubarSeparator />
          <MenubarItem>Print</MenubarItem>
        </MenubarContent>
      </MenubarMenu>
      <MenubarContent>
        <Alert v-if="!device" variant="destructive">
          <AlertTitle>Error</AlertTitle>
          <AlertDescription>
            Device not connected
          </AlertDescription>
        </Alert>
      </MenubarContent>
    </Menubar>

    <div class="status">
      <div v-if="device">
        '{{ device.name }}' connected successfully
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import {
  Menubar, MenubarContent,
  MenubarItem,
  MenubarMenu, MenubarSeparator,
  MenubarShortcut, MenubarTrigger,
} from '@/components/ui/menubar';

const {
  device
} = defineProps<{
  device: BluetoothDevice,
}>()

</script>

<style lang="scss">
.status {
  display: block;
  position: fixed;
  top: 0;
  left: 0;
  box-shadow: 0 -5px 1px solid var(--primary);
  width: 100vw;
  box-sizing: border-box;
  text-align: center;
  font-size: 20px;
}

.menubar {
  position: fixed;
  top: 0;
  left: 0;
}
</style>
