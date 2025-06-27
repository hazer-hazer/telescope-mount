<template>
  <header class="sticky top-0 z-10 h-12 flex border-b bg-background px-4 items-center justify-between">
    <span class="text-xl font-bold font-stretch-expanded select-none">
      Telemount app
    </span>

    <DropdownMenu>
      <DropdownMenuTrigger as-child>
        <Button variant="outline" class="px-3">
          <Icon name="lucide-sun"
            class="h-[1.2rem] w-[1.2rem] rotate-0 scale-100 transition-all dark:-rotate-90 dark:scale-0" />
          <Icon name="lucide-moon"
            class="absolute h-[1.2rem] w-[1.2rem] rotate-90 scale-0 transition-all dark:rotate-0 dark:scale-100" />
          <span class="sr-only">Toggle theme</span>
        </Button>
      </DropdownMenuTrigger>
      <DropdownMenuContent align="end">
        <DropdownMenuItem @click="colorMode.preference = 'light'">
          <Icon name="lucide-sun" />
          Light
          <Icon v-if="colorMode.preference === 'light'" name="lucide-check" class="ml-auto" />
        </DropdownMenuItem>
        <DropdownMenuItem @click="colorMode.preference = 'dark'">
          <Icon name="lucide-moon" />
          Dark
          <Icon v-if="colorMode.preference === 'dark'" name="lucide-check" class="ml-auto" />
        </DropdownMenuItem>
        <DropdownMenuItem @click="colorMode.preference = 'system'">
          <Icon name="lucide-computer" />
          System
          <Icon v-if="colorMode.preference === 'system'" name="lucide-check" class="ml-auto" />
        </DropdownMenuItem>
      </DropdownMenuContent>
    </DropdownMenu>
  </header>
</template>

<script lang="ts" setup>
const colorMode = useColorMode()
</script>

<style scoped lang="scss"></style>