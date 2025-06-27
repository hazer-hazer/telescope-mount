<template>
  <TooltipProvider>
    <Tooltip>
      <TooltipTrigger as-child>
        <Badge :variant="badgeVariant" :class="badgeClass">
          {{ props.kind }}
        </Badge>
      </TooltipTrigger>
      <TooltipContent v-if="error">
        <p>{{ error }}</p>
      </TooltipContent>
    </Tooltip>
  </TooltipProvider>
</template>

<script lang="ts" setup>
import { ConnectionState } from '~/types/connection';

const props = defineProps<{
  kind: ConnectionState
  error?: unknown | null
}>()

const badgeVariant = computed(() => {
  if (props.error) {
    return 'destructive'
  }

  switch (props.kind) {
    case ConnectionState.Connecting:
      return 'default';
    case ConnectionState.Ready:
      return 'secondary';
    case ConnectionState.Error:
      return 'destructive';
    default:
      return 'outline';
  }
});

const badgeClass = computed(() => {
  switch (props.kind) {
    case ConnectionState.Connected:
      return 'border-emerald-300 text-emerald-500'
    default: return null
  }
})

</script>

<style scoped lang="scss"></style>
